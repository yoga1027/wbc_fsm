#include "FSM/FSM.h"
#include "interface/UserCommandMapping.h"
#include <limits>
#include <stdexcept>

// Exercise the production FSM scheduling and command flush without DDS.
class TestIO : public IOInterface
{
public:
    TestIO() { cmdPanel = nullptr; }
    LowlevelState sample{};
    LowlevelCmd lastSent{};
    int sends = 0;
    RecoveryEntryLatch entryLatch;
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) override
    {
        lastSent = *cmd;
        *state = sample;
        state->heldUserCmd = sample.userCmd;
        state->userCmd = entryLatch.sample(sample.userCmd);
        ++sends;
    }
};

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    try {
        CtrlComponents ctrl(nullptr);
        TestIO io;
        ctrl.ioInter = &io;
        // IOInterface has a non-virtual destructor; keep the borrowed object on the stack.
        struct Detach { CtrlComponents &ctrl; ~Detach() { ctrl.ioInter = nullptr; } } detach{ctrl};
        ctrl.dt = 0.02;
        io.sample.imu.quaternion[0] = 1.0f;
        io.sample.userCmd = UserCommand::NONE;
        FSM fsm(&ctrl);
        auto step = [&](UserCommand command) {
            io.sample.userCmd = command;
            fsm.run();
            require(!ctrl.exitFlag, "Unexpected FSM exit");
        };
        step(UserCommand::R2_X);
        step(UserCommand::R2_X);
        step(UserCommand::R2_X);
        step(UserCommand::R2_Y);
        require(ctrl.lowCmd->motorCmd[0].Kp == 0 && io.lastSent.motorCmd[0].Kp == 0,
                "Held entry/start activated recovery without release");
        step(UserCommand::NONE);
        for (int i = 0; i < 30; ++i) {
            step(i % 2 ? UserCommand::R2_A : UserCommand::R2_B);
            require(ctrl.lowCmd->motorCmd[0].Kp == 0 && io.lastSent.motorCmd[0].Kp == 0,
                    "Waiting emitted policy commands or switched to an old policy");
        }
        step(UserCommand::R2_Y);
        require(ctrl.lowCmd->motorCmd[0].Kp > 99 && ctrl.lowCmd->motorCmd[0].Kp < 100,
                "Fresh R2+Y did not start recovery");
        for (int i = 0; i < 28; ++i) step(UserCommand::NONE);
        step(UserCommand::R2_B);
        step(UserCommand::NONE);
        require(ctrl.lowCmd->motorCmd[0].Kp == 200, "Recovery did not switch to LOCO");
        step(UserCommand::R2_X);
        step(UserCommand::NONE);
        require(ctrl.lowCmd->motorCmd[0].Kp == 0, "Recovery re-entry skipped waiting");
        step(UserCommand::R2_Y);
        require(ctrl.lowCmd->motorCmd[0].Kp < 100 && ctrl.lowCmd->motorCmd[0].Kp > 99,
                "LOCO did not return to recovery");
        io.sample.motorState[0].q = std::numeric_limits<float>::quiet_NaN();
        step(UserCommand::NONE);
        require(ctrl.lowCmd->motorCmd[0].Kp == 0, "Fault kept active command in buffer");
        step(UserCommand::NONE);
        require(io.lastSent.motorCmd[0].Kp == 0 && io.lastSent.motorCmd[0].Kd == 10,
                "Fault damping was not sent on the next IO cycle");
        io.sample.motorState[0].q = 0;
        step(UserCommand::R2_X);
        step(UserCommand::R2_Y);
        step(UserCommand::R2_Y);
        require(ctrl.lowCmd->motorCmd[0].Kp == 0, "Start held during re-entry was accepted");
        step(UserCommand::NONE);
        step(UserCommand::R2_Y);
        require(ctrl.lowCmd->motorCmd[0].Kp > 0, "Recovery did not restart after release");
        const int previousSends = io.sends;
        io.sample.userCmd = UserCommand::SELECT;
        fsm.run();
        require(ctrl.exitFlag, "SELECT did not stop FSM");
        require(io.sends == previousSends + 2, "SELECT did not flush damping before loop exit");
        for (const auto &cmd : io.lastSent.motorCmd)
            require(cmd.Kp == 0 && cmd.Kd == 10 && cmd.dq == 0 && cmd.tau == 0,
                    "SELECT left an active policy target on IO");
        std::cout << "PASS: production FSM transitions and damping publication with fake IO" << std::endl;
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << std::endl;
        return 1;
    }
}
