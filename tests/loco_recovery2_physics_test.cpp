#include "FSM/State_LocoRecovery2.h"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

// Isolated headless physics and the production ONNX policies. This creates no
// DDS participant and does not interact with any running robot or simulator.
void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}

void requireUnpowered(const LowlevelCmd &command)
{
    for (const auto &motor : command.motorCmd)
        require(motor.q == 0 && motor.dq == 0 && motor.tau == 0 &&
                motor.Kp == 0 && motor.Kd == 0,
                "Protection wait retained joint support or responded to a walking command");
}

struct PushCase { const char *name; double forceX, forceY; };

void runCase(const PushCase &push)
{
    using Phase = State_LocoRecovery2::Phase;
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(RECOVERY_TEST_SCENE, nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) throw std::runtime_error(error);
    require(model->nu == 29 && model->nq == 36,
            "Expected the floating-base G1 29-DOF scene");
    model->opt.timestep = 0.002;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        mj_makeData(model.get()), mj_deleteData);
    const float initial[29] = {
        -0.312, 0, 0, 0.669, -0.363, 0,
        -0.312, 0, 0, 0.669, -0.363, 0, 0, 0, 0,
        0.2, 0.2, 0, 0.6, 0, 0, 0,
        0.2, -0.2, 0, 0.6, 0, 0, 0};
    for (int i = 0; i < 29; ++i) data->qpos[7 + i] = initial[i];
    data->qpos[2] = 0.75;
    mj_forward(model.get(), data.get());
    auto sensorAddress = [&](const char *name) {
        const int id = mj_name2id(model.get(), mjOBJ_SENSOR, name);
        if (id < 0) throw std::runtime_error(std::string("Missing sensor: ") + name);
        return model->sensor_adr[id];
    };
    const int quaternion = sensorAddress("imu_quat"), gyro = sensorAddress("imu_gyro");
    const int pelvis = mj_name2id(model.get(), mjOBJ_BODY, "pelvis");
    const int torso = mj_name2id(model.get(), mjOBJ_BODY, "torso_link");
    require(pelvis >= 0 && torso >= 0, "Missing pelvis/torso body");
    CtrlComponents ctrl(nullptr);
    ctrl.dt = 0.02;
    State_Loco loco(&ctrl);
    State_MJAMP_RECOVERY recovery(&ctrl);
    State_LocoRecovery2 state(&ctrl, loco, recovery);
    auto feedback = [&] {
        for (int i = 0; i < 29; ++i) {
            ctrl.lowState->motorState[i].q = data->sensordata[i];
            ctrl.lowState->motorState[i].dq = data->sensordata[29 + i];
        }
        for (int i = 0; i < 4; ++i)
            ctrl.lowState->imu.quaternion[i] = data->sensordata[quaternion + i];
        for (int i = 0; i < 3; ++i)
            ctrl.lowState->imu.gyroscope[i] = data->sensordata[gyro + i];
    };
    feedback();
    state.enter();
    bool protectionSeen = false, activationSent = false, recoverySeen = false;
    bool blendSeen = false, returned = false, walkingStarted = false;
    bool walkingLocoSustained = true;
    int protectionFrames = 0;
    double protectionTime = 0, recoveryTime = 0, returnTime = 0;
    double minimumHeight = 1, activationHeight = 0, walkX = 0, walkY = 0;
    for (int tick = 0; tick < 2250; ++tick) {
        feedback();
        const double time = data->time;
        ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = UserCommand::NONE;
        ctrl.lowState->userValue.setZero();
        if (state.phase() == Phase::PROTECT_WAIT && !activationSent) {
            const double wait = time - protectionTime;
            // A held R1 must be released before a fresh press can start Recovery.
            // Nonzero walking input must have no effect during protection.
            ctrl.lowState->userValue.ly = 0.6;
            if (wait < 1.0) {
                ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = UserCommand::R1;
            } else if (wait >= 3.0) {
                ctrl.lowState->userValue.setZero();
                ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = UserCommand::R1;
                activationSent = true;
                activationHeight = data->xpos[3 * pelvis + 2];
            }
        }
        if (returned && time >= returnTime + 0.7) {
            if (!walkingStarted) {
                walkingStarted = true;
                walkX = data->xpos[3 * pelvis];
                walkY = data->xpos[3 * pelvis + 1];
            }
            ctrl.lowState->userValue.ly = 0.3;
        }
        state.run();
        require(state.checkChange() == FSMStateName::LOCO_RECOVERY2,
                "Physical run exited the new composite task");
        if (state.phase() == Phase::PROTECT_WAIT) {
            if (!protectionSeen) {
                protectionSeen = true;
                protectionTime = time;
            }
            requireUnpowered(*ctrl.lowCmd);
            ++protectionFrames;
        }
        if (protectionSeen && !activationSent)
            require(state.phase() == Phase::PROTECT_WAIT,
                    "Recovery started automatically or accepted a held R1 before release");
        if (state.phase() == Phase::RECOVERY && !recoverySeen) {
            require(activationSent, "Recovery started without a fresh R1 activation");
            recoverySeen = true;
            recoveryTime = time;
        }
        if (recoverySeen && state.phase() == Phase::RETURNING) blendSeen = true;
        if (blendSeen && state.phase() == Phase::LOCO && !returned) {
            returned = true;
            returnTime = time;
        }
        if (returned && state.phase() != Phase::LOCO) walkingLocoSustained = false;
        if (time > 3 && !returned)
            minimumHeight = std::min(minimumHeight, data->xpos[3 * pelvis + 2]);
        for (int substep = 0; substep < 10; ++substep) {
            for (int i = 0; i < 29; ++i) {
                const auto &command = ctrl.lowCmd->motorCmd[i];
                data->ctrl[i] = command.tau + command.Kp *
                    (command.q - data->sensordata[i]) + command.Kd *
                    (command.dq - data->sensordata[29 + i]);
            }
            const bool pushing = data->time >= 3 && data->time < 3.5;
            data->xfrc_applied[6 * torso] = pushing ? push.forceX : 0;
            data->xfrc_applied[6 * torso + 1] = pushing ? push.forceY : 0;
            mj_step(model.get(), data.get());
        }
        if ((returned && time >= returnTime + 5.7) ||
            (recoverySeen && !returned && time >= recoveryTime + 25.0)) break;
    }
    const double distance = walkingStarted ? std::hypot(
        data->xpos[3 * pelvis] - walkX, data->xpos[3 * pelvis + 1] - walkY) : 0;
    const double finalHeight = data->xpos[3 * pelvis + 2];
    const bool finalLoco = state.phase() == Phase::LOCO;
    std::cout << "RESULT " << push.name << ": protection=" << protectionTime
              << "s, unpowered_frames=" << protectionFrames << ", R1/recovery="
              << recoveryTime << "s, return=" << returnTime
              << "s, minimum_pelvis_height=" << minimumHeight
              << "m, R1_pelvis_height=" << activationHeight
              << "m, walk_distance=" << distance
              << "m, final_pelvis_height=" << finalHeight << "m\n";
    state.exit();
    require(protectionSeen && protectionFrames >= 150,
            "Physical push did not exercise at least three seconds of unpowered protection");
    require(recoverySeen && recoveryTime - protectionTime >= 2.99,
            "Fresh R1 did not start Recovery after the protection wait");
    require(minimumHeight < 0.4 && activationHeight < 0.4,
            "The robot did not physically fall before manual Recovery activation");
    require(returned && returnTime - recoveryTime < 25.0,
            "Manual Recovery did not return to Loco within 25 seconds");
    require(finalLoco && walkingLocoSustained && distance > 0.5 && finalHeight > 0.65,
            "Recovered Loco did not remain upright and walk in response to joystick input");
}

int main()
{
    const std::array<PushCase, 3> cases{{
        {"front_250N_manual_recovery", 250, 0},
        {"back_250N_manual_recovery", -250, 0},
        {"side_250N_manual_recovery", 0, 250}}};
    int failures = 0;
    for (const auto &push : cases) {
        try {
            runCase(push);
            std::cout << "PASS: " << push.name << '\n';
        } catch (const std::exception &error) {
            ++failures;
            std::cerr << "FAIL: " << push.name << ": " << error.what() << '\n';
        }
    }
    if (failures == 0)
        std::cout << "PASS: three physical fall/protection/manual get-up/return/walking cases "
                     "(headless MuJoCo, production ONNX, no DDS).\n";
    return failures == 0 ? 0 : 1;
}
