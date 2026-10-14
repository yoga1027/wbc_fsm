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
    for (float &v : s.imu.gyroscope) v = 0;
    for (auto &m : s.motorState) { m.q = 0; m.dq = 0; }
    for (int offset : {0, 6}) {
        s.motorState[offset].q = -0.312;
        s.motorState[offset + 3].q = 0.669;
        s.motorState[offset + 4].q = -0.363;
    }
}
void protection(const LowlevelCmd &cmd)
{
    for (const auto &m : cmd.motorCmd)
        require(m.mode == 10 && m.Kp == 0 && m.Kd == 0 && m.q == 0 && m.dq == 0 && m.tau == 0,
                "High impedance wait left an active motor command");
}
int main()
{
    try {
        using Pad = unitree::common::Gamepad;
        Pad pad;
        pad.R1.update(true);
        require(userCommandFromGamepad(pad, true) == UserCommand::R1,
                "Strict decoder rejected standalone R1");
        // Cover all physical button bits, including the combinations that the
        // original decoder intentionally treats as ordinary R1 (down, A, etc.).
        for (auto other : {&Pad::L1, &Pad::start, &Pad::select, &Pad::R2,
                           &Pad::L2, &Pad::F1, &Pad::F2, &Pad::A,
                           &Pad::B, &Pad::X, &Pad::Y, &Pad::up,
                           &Pad::right, &Pad::down, &Pad::left}) {
            pad = Pad{};
            pad.R1.update(true);
            (pad.*other).update(true);
            const auto decoded = userCommandFromGamepad(pad, true);
            require(decoded != UserCommand::R1 && decoded != UserCommand::NONE,
                    "Strict decoder lost a held R1 combination or accepted it as standalone R1");
        }
        pad = Pad{}; pad.R1.update(true); pad.down.update(true);
        require(userCommandFromGamepad(pad) == UserCommand::R1 &&
                userCommandFromGamepad(pad, true) == UserCommand::R1_OTHER,
                "Strict R1 handling changed the original task decoder or lost R1+down");
        pad.select.update(true);
        require(userCommandFromGamepad(pad, true) == UserCommand::SELECT,
                "Strict R1 handling suppressed SELECT priority");
        pad.select.update(false); pad.L2.update(true); pad.B.update(true);
        require(userCommandFromGamepad(pad, true) == UserCommand::L2_B,
                "Strict R1 handling suppressed manual-stop priority");

        CtrlComponents ctrl(nullptr);
        TestIO io;
        ctrl.ioInter = &io;
        struct Detach { CtrlComponents &ctrl; ~Detach() { ctrl.ioInter = nullptr; } } detach{ctrl};
        ctrl.dt = 0.02;
        ctrl.locoRecovery2 = true;
        standing(io);
        FSM fsm(&ctrl);
        auto step = [&](UserCommand button = UserCommand::NONE) {
            io.sample.userCmd = button;
            fsm.run();
            require(!ctrl.exitFlag, "Unexpected second-task FSM exit");
        };
        auto inTask = [&] {
            require(fsm.currentStateName() == FSMStateName::LOCO_RECOVERY2,
                    "Second task fell through to another outer FSM mode");
        };
        step(); step(UserCommand::R1);
        require(fsm.currentStateName() == FSMStateName::PASSIVE && io.lastSent.motorCmd[0].Kp == 0,
                "R1 outside protection auto-started motors");
        step(UserCommand::START); step(UserCommand::L2_B);
        require(fsm.currentStateName() == FSMStateName::PASSIVE && ctrl.lowCmd->motorCmd[0].Kp == 0,
                "Stop did not cancel pending fixed stand in task 2");
        step(UserCommand::START); step(); step(UserCommand::R2_B);
        require(fsm.currentStateName() == FSMStateName::FIXEDSTAND,
                "Second-task entry bypassed fixed stand completion");
        for (int i = 0; i < 102; ++i) step();
        step(UserCommand::R2_B); step(); inTask(); step();
        require(io.lastSent.motorCmd[0].Kp == 200, "Second task did not publish Loco before the fall");

        // The fall replaces the outgoing buffer immediately. With the real FSM
        // send-before-compute order, IO publishes protection on its next cycle.
        standing(io, 1.1); step(UserCommand::R1); inTask();
        require(io.lastSent.motorCmd[0].Kp == 200, "Detection changed an already published Loco command");
        protection(*ctrl.lowCmd);
        step(UserCommand::R1); protection(io.lastSent); protection(*ctrl.lowCmd);
        for (int i = 0; i < 15; ++i) { step(UserCommand::R1); inTask(); protection(*ctrl.lowCmd); }
        step(UserCommand::R2_X); step(UserCommand::R2_X);
        require(ctrl.lowState->userCmd == UserCommand::NONE &&
                ctrl.lowState->heldUserCmd == UserCommand::R2_X,
                "Test IO did not reproduce the raw held recovery key latch");
        step(UserCommand::R1); inTask(); protection(*ctrl.lowCmd);
        step(UserCommand::NONE);
        for (auto button : {UserCommand::R1_UP, UserCommand::R1_LEFT,
                           UserCommand::R1_RIGHT, UserCommand::R2_Y,
                           UserCommand::R2_B, UserCommand::START}) {
            step(button); inTask(); protection(*ctrl.lowCmd);
        }
        step();
        pad = Pad{}; pad.R1.update(true); pad.down.update(true);
        step(userCommandFromGamepad(pad, true)); inTask(); protection(*ctrl.lowCmd);
        pad.down.update(false);
        step(userCommandFromGamepad(pad, true)); inTask(); protection(*ctrl.lowCmd);
        pad.R1.update(false); step(userCommandFromGamepad(pad, true));
        pad.R1.update(true); step(userCommandFromGamepad(pad, true)); inTask();
        protection(io.lastSent);
        require(ctrl.lowCmd->motorCmd[0].Kp > 0, "Fresh R1 did not prepare Recovery in its activation cycle");
        step(UserCommand::R1);
        require(io.lastSent.motorCmd[0].Kp > 0, "R1 Recovery action was not published next cycle");
        for (int i = 0; i < 20; ++i) {
            step(UserCommand::R1); inTask();
            require(ctrl.lowCmd->motorCmd[0].Kp > 0, "Fallen get-up feedback re-entered protection");
        }
        standing(io);
        for (int i = 0; i < 72; ++i) step(UserCommand::R1);
        inTask();
        require(io.lastSent.motorCmd[0].Kp == 200, "Get-up failed to transition back to Loco");

        // A repeat fall needs a second release/new press; authorization is not sticky.
        standing(io, 1.1); step(UserCommand::R1); step(UserCommand::R1);
        protection(io.lastSent); protection(*ctrl.lowCmd);
        step(); step(UserCommand::R1);
        require(ctrl.lowCmd->motorCmd[0].Kp > 0, "Second authorized get-up failed to activate");
        standing(io); for (int i = 0; i < 72; ++i) step(); inTask();

        // Manual recovery remains selectable from Loco. Its exit to LOCO must
        // resolve to task 2, and entry while non-standing must stay unloaded.
        step(UserCommand::R2_X); step(UserCommand::L2_B);
        require(fsm.currentStateName() == FSMStateName::PASSIVE && ctrl.lowCmd->motorCmd[0].Kp == 0,
                "Stop did not cancel a pending manual recovery transition");
        step(UserCommand::R2_X); step();
        require(fsm.currentStateName() == FSMStateName::MJAMP_RECOVERY &&
                ctrl.lowCmd->motorCmd[0].Kp == 0 && ctrl.lowCmd->motorCmd[0].Kd == 10,
                "Independent manual Recovery lost its damping wait");
        step(UserCommand::R2_Y); for (int i = 0; i < 28; ++i) step();
        step(UserCommand::R2_B); step(); inTask();
        io.sample.motorState[0].dq = std::numeric_limits<float>::quiet_NaN();
        step();
        require(ctrl.lowCmd->motorCmd[0].Kp == 0 && ctrl.lowCmd->motorCmd[0].Kd == 10,
                "Invalid feedback left an active motor buffer");
        step();
        require(fsm.currentStateName() == FSMStateName::PASSIVE &&
                io.lastSent.motorCmd[0].Kp == 0 && io.lastSent.motorCmd[0].Kd == 10,
                "Fault did not publish passive damping");
        standing(io); step(UserCommand::R1); step(UserCommand::R2_B);
        require(fsm.currentStateName() == FSMStateName::PASSIVE, "Fault automatically re-entered task 2");

        step(UserCommand::R2_X); step(); step(UserCommand::R2_Y);
        for (int i = 0; i < 28; ++i) step();
        step(UserCommand::R2_B); standing(io, 0.3); step(); inTask();
        protection(*ctrl.lowCmd); step(); protection(io.lastSent);
        const int beforeShutdown = io.sends;
        io.sample.userCmd = UserCommand::SELECT;
        fsm.run();
        require(ctrl.exitFlag && io.sends == beforeShutdown + 2,
                "SELECT did not flush passive damping before shutdown");
        for (const auto &m : io.lastSent.motorCmd)
            require(m.Kp == 0 && m.Kd == 10 && m.q == 0 && m.dq == 0 && m.tau == 0,
                    "SELECT left high-impedance/policy buffer instead of passive damping");
        std::cout << "PASS: task 2 routing/startup, protection publication/wait, release/exact R1, "
                     "held recovery keys, get-up/repeated return, manual routing, "
                     "pending-transition stop, invalid feedback and SELECT flush (fake IO)\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
