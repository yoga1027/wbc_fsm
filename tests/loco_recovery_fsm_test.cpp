#include "FSM/FSM.h"
#include "interface/UserCommandMapping.h"
#include <cmath>
#include <limits>
#include <stdexcept>

class TestIO : public IOInterface
{
public:
    TestIO() { cmdPanel = nullptr; }
    LowlevelState sample{};
    LowlevelCmd lastSent{};
    RecoveryEntryLatch latch;
    int sends = 0;
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) override
    {
        lastSent = *cmd;
        *state = sample;
        state->heldUserCmd = sample.userCmd;
        state->userCmd = latch.sample(sample.userCmd);
        ++sends;
    }
};
void require(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
void standing(TestIO &io, float tilt = 0)
{
    auto &s = io.sample;
    s.imu.quaternion[0] = std::cos(tilt / 2);
    s.imu.quaternion[1] = s.imu.quaternion[3] = 0;
    s.imu.quaternion[2] = std::sin(tilt / 2);
    for (auto &m : s.motorState) { m.q = 0; m.dq = 0; }
    for (int offset : {0, 6}) {
        s.motorState[offset].q = -0.312;
        s.motorState[offset + 3].q = 0.669;
        s.motorState[offset + 4].q = -0.363;
    }
}
int main()
{
    try {
        CtrlComponents ctrl(nullptr);
        TestIO io;
        ctrl.ioInter = &io;
        struct Detach { CtrlComponents &ctrl; ~Detach() { ctrl.ioInter = nullptr; } } detach{ctrl};
        ctrl.dt = 0.02;
        ctrl.locoAutoRecovery = true;
        standing(io);
        FSM fsm(&ctrl);
        auto step = [&](UserCommand command = UserCommand::NONE) {
            io.sample.userCmd = command;
            fsm.run();
            require(!ctrl.exitFlag, "Unexpected FSM exit");
        };
        step();
        require(fsm.currentStateName() == FSMStateName::PASSIVE &&
                io.lastSent.motorCmd[0].Kp == 0, "New task auto-started motors");
        step(UserCommand::START);
        step(UserCommand::L2_B);
        require(fsm.currentStateName() == FSMStateName::PASSIVE &&
                ctrl.lowCmd->motorCmd[0].Kp == 0, "Stop did not cancel pending fixed stand");
        step(UserCommand::START); step();
        step(UserCommand::R2_B);
        require(fsm.currentStateName() == FSMStateName::FIXEDSTAND,
                "Combined entry bypassed fixed stand completion");
        for (int i = 0; i < 102; ++i) step();
        step(UserCommand::R2_B); step();
        require(fsm.currentStateName() == FSMStateName::LOCO_RECOVERY,
                "New task did not enter composite from fixed stand");
        step();
        require(io.lastSent.motorCmd[0].Kp == 200, "Loco not sent before disturbance");
        standing(io, 0.8);
        step();
        require(fsm.currentStateName() == FSMStateName::LOCO_RECOVERY &&
                ctrl.lowCmd->motorCmd[0].Kp > 0 && ctrl.lowCmd->motorCmd[0].Kp < 100,
                "Detection cycle did not prepare recovery command");
        step();
        require(io.lastSent.motorCmd[0].Kp > 0 && io.lastSent.motorCmd[0].Kp < 100,
                "Recovery was not published on the next IO cycle");
        standing(io);
        // A mode key pressed during recovery cannot replay when return finishes.
        for (int i = 0; i < 72; ++i) step(UserCommand::R2_A);
        require(fsm.currentStateName() == FSMStateName::LOCO_RECOVERY &&
                io.lastSent.motorCmd[0].Kp == 200,
                "Auto return exited composite or replayed held mode request");
        step(); step(UserCommand::R2_A); step(UserCommand::L2_B);
        require(fsm.currentStateName() == FSMStateName::PASSIVE &&
                ctrl.lowCmd->motorCmd[0].Kp == 0, "Stop lost to pending policy transition");

        // Independent manual recovery remains available; its Loco exit is routed
        // back to composite only in this opt-in task.
        step(UserCommand::R2_X); step();
        require(fsm.currentStateName() == FSMStateName::MJAMP_RECOVERY &&
                ctrl.lowCmd->motorCmd[0].Kp == 0, "Manual recovery skipped waiting");
        step(UserCommand::R2_Y);
        for (int i = 0; i < 28; ++i) step();
        step(UserCommand::R2_B); step();
        require(fsm.currentStateName() == FSMStateName::LOCO_RECOVERY,
                "Manual recovery Loco exit bypassed composite routing");
        io.sample.motorState[0].q = std::numeric_limits<float>::quiet_NaN();
        step();
        require(ctrl.lowCmd->motorCmd[0].Kp == 0, "Fault left active buffer");
        step();
        require(fsm.currentStateName() == FSMStateName::PASSIVE &&
                io.lastSent.motorCmd[0].Kp == 0, "Fault damping not sent");
        standing(io);
        step(UserCommand::R2_B);
        require(fsm.currentStateName() == FSMStateName::PASSIVE, "Fault restarted automatically");
        step(UserCommand::R2_X); step(); step(UserCommand::R2_Y);
        for (int i = 0; i < 28; ++i) step();
        step(UserCommand::R2_B); step();
        const int previousSends = io.sends;
        io.sample.userCmd = UserCommand::SELECT;
        fsm.run();
        require(ctrl.exitFlag && io.sends == previousSends + 2,
                "SELECT did not flush damping before exit");
        for (const auto &cmd : io.lastSent.motorCmd)
            require(cmd.Kp == 0 && cmd.Kd == 10 && cmd.dq == 0 && cmd.tau == 0,
                    "SELECT left active motor commands");
        std::cout << "PASS: composite startup, automatic takeover/publication/return, held requests, "
                     "manual routing, fault and pending-transition stop (fake IO)\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
