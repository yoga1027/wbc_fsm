#include "interface/IOSDK.h"
#include "interface/MotionServiceHandoff.h"
#include <cmath>
#include <net/if.h>
#include <stdio.h>
#include <iostream>
#include <thread>

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

IOSDK::IOSDK(const StartupOptions &options, const std::function<bool()> &cancelled)
    : strictStandaloneR1_(options.locoRecovery2)
{
    if (if_nametoindex(options.networkInterface.c_str()) == 0)
        throw std::runtime_error("Network interface does not exist: " + options.networkInterface);
    userValue_.setZero();
    std::cout << "[Startup] " << (options.realRobot ? "REAL ROBOT" : "SIMULATION")
              << ", interface=" << options.networkInterface
              << ", DDS domain=" << options.domainId() << std::endl;
    ChannelFactory::Instance()->Init(options.domainId(), options.networkInterface);

    // Receive feedback, but do not wait for it before releasing the stock
    // service. The command publisher is created only after verified handoff.
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    try {
        lowstate_subscriber_->InitChannel(std::bind(&IOSDK::LowStateHandler, this, std::placeholders::_1), 1);
        if (options.realRobot) {
            motionClient_ = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
            motionClient_->Init();
        }
        prepareMotionControl(options.realRobot,
            [&](std::string &form, std::string &name, std::chrono::milliseconds budget) {
                motionClient_->SetTimeout(static_cast<float>(budget.count()) / 1000.0f);
                return motionClient_->CheckMode(form, name);
            },
            [&](std::chrono::milliseconds budget) {
                motionClient_->SetTimeout(static_cast<float>(budget.count()) / 1000.0f);
                return motionClient_->ReleaseMode();
            },
            [&] { return hasFreshState(); },
            [&] {
                lowcmd_publisher_ = std::make_shared<ChannelPublisher<LowCmd_>>(HG_CMD_TOPIC);
                lowcmd_publisher_->InitChannel();
                controlReady_ = true;
            },
            [&](std::chrono::milliseconds duration) {
                const auto until = std::chrono::steady_clock::now() + duration;
                while (!cancelled() && std::chrono::steady_clock::now() < until)
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
            },
            [] { return std::chrono::steady_clock::now(); }, cancelled, std::cout);
        std::cout << "[Startup] Ready. Starting in PASSIVE damping; no START required."
                  << std::endl;
    } catch (...) {
        lowstate_subscriber_->CloseChannel();
        throw;
    }
}

IOSDK::~IOSDK()
{
    lowstate_subscriber_->CloseChannel();
}

bool IOSDK::hasFreshState()
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return stateReady_ && std::chrono::steady_clock::now() - lastStateTime_ < std::chrono::milliseconds(250);
}

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    if (!controlReady_) throw std::runtime_error("Low-level control has not been authorized by startup checks");
    uint8_t modeMachine;
    {
        // One coherent snapshot for joints, IMU, held buttons and robot type.
        std::lock_guard<std::mutex> lock(stateMutex_);
        *state = _lowState;
        state->heldUserCmd = userCmd_;
        state->userCmd = recoveryEntryLatch_.sample(userCmd_);
        state->userValue = userValue_;
        modeMachine = mode_machine_;
    }
    // send control cmd
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(Mode::PR);
    dds_low_command.mode_machine() = modeMachine;
    for (size_t i = 0; i < G1_NUM_MOTOR; i++)
    {
        
        dds_low_command.motor_cmd().at(i).mode() = 1; // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = cmd->motorCmd[i].tau;
        dds_low_command.motor_cmd().at(i).q() = cmd->motorCmd[i].q;
        dds_low_command.motor_cmd().at(i).dq() = cmd->motorCmd[i].dq;
        dds_low_command.motor_cmd().at(i).kp() = cmd->motorCmd[i].Kp;
        dds_low_command.motor_cmd().at(i).kd() = cmd->motorCmd[i].Kd;
        // std::cout<<"des_q: "<<dds_low_command.motor_cmd().at(i).q()<<std::endl;
    }

    dds_low_command.crc() = crc32_core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
    if (!lowcmd_publisher_->Write(dds_low_command))
        throw std::runtime_error("Failed to publish rt/lowcmd");
}

void IOSDK::LowStateHandler(const void *message)
{
    LowState_ low_state = *(const LowState_ *)message;
    if (low_state.crc() != crc32_core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1))
    {
        std::cout << "[ERROR] CRC Error" << std::endl;
        return;
    }

    // CRC alone does not make a frame usable for policy initialization.
    float normSquared = 0;
    for (float v : low_state.imu_state().quaternion()) normSquared += v * v;
    bool valid = std::isfinite(normSquared) && normSquared >= 0.25f && normSquared <= 2.25f &&
                 low_state.mode_machine() != 0;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        if (!std::isfinite(low_state.motor_state()[i].q()) ||
            !std::isfinite(low_state.motor_state()[i].dq())) valid = false;
    }
    for (float v : low_state.imu_state().gyroscope())
        if (!std::isfinite(v)) valid = false;
    for (float v : low_state.imu_state().accelerometer())
        if (!std::isfinite(v)) valid = false;
    std::lock_guard<std::mutex> lock(stateMutex_);

    // get motor state
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        _lowState.motorState[i].q = low_state.motor_state()[i].q();
        _lowState.motorState[i].dq = low_state.motor_state()[i].dq();
    }
    
    // get imu state
    _lowState.imu.gyroscope[0] = low_state.imu_state().gyroscope()[0];
    _lowState.imu.gyroscope[1] = low_state.imu_state().gyroscope()[1];
    _lowState.imu.gyroscope[2] = low_state.imu_state().gyroscope()[2];

    _lowState.imu.quaternion[0] = low_state.imu_state().quaternion()[0];
    _lowState.imu.quaternion[1] = low_state.imu_state().quaternion()[1];
    _lowState.imu.quaternion[2] = low_state.imu_state().quaternion()[2];
    _lowState.imu.quaternion[3] = low_state.imu_state().quaternion()[3];

    _lowState.imu.accelerometer[0] = low_state.imu_state().accelerometer()[0];
    _lowState.imu.accelerometer[1] = low_state.imu_state().accelerometer()[1];
    _lowState.imu.accelerometer[2] = low_state.imu_state().accelerometer()[2];

    // update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    gamepad_.update(rx_.RF_RX);

    // update mode machine
    if (mode_machine_ != low_state.mode_machine())
    {
        if (mode_machine_ == 0)
            std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
        mode_machine_ = low_state.mode_machine();
    }

    userCmd_ = userCommandFromGamepad(gamepad_, strictStandaloneR1_);

    userValue_.lx = -gamepad_.lx;
    userValue_.ly = gamepad_.ly;
    userValue_.rx = -gamepad_.rx;
    userValue_.ry = gamepad_.ry;
    // Keep runtime faults visible to the active state, rather than hiding them
    // behind an older valid observation. Only startup readiness uses this flag.
    stateReady_ = valid;
    lastStateTime_ = std::chrono::steady_clock::now();
}
