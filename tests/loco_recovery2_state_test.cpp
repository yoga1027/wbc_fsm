#include "FSM/State_LocoRecovery2.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using Phase = State_LocoRecovery2::Phase;
void require(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }

void pose(CtrlComponents &ctrl, float tilt = 0, bool roll = false)
{
    auto &s = *ctrl.lowState;
    s.userCmd = s.heldUserCmd = UserCommand::NONE;
    s.userValue.setZero();
    s.imu.quaternion[0] = std::cos(tilt / 2);
    s.imu.quaternion[1] = roll ? std::sin(tilt / 2) : 0;
    s.imu.quaternion[2] = roll ? 0 : std::sin(tilt / 2);
    s.imu.quaternion[3] = 0;
    for (float &v : s.imu.gyroscope) v = 0;
    for (auto &m : s.motorState) { m.q = 0; m.dq = 0; }
    for (int offset : {0, 6}) {
        s.motorState[offset].q = -0.312;
        s.motorState[offset + 3].q = 0.669;
        s.motorState[offset + 4].q = -0.363;
    }
}
void command(CtrlComponents &ctrl, UserCommand value)
{
    ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = value;
}
void protection(const LowlevelCmd &cmd)
{
    for (const auto &m : cmd.motorCmd)
        require(m.mode == 10 && m.Kp == 0 && m.Kd == 0 && m.q == 0 && m.dq == 0 && m.tau == 0,
                "Protection left a position target, stiffness, damping, or feedforward active");
}
void damping(const LowlevelCmd &cmd)
{
    for (const auto &m : cmd.motorCmd)
        require(m.mode == 10 && m.Kp == 0 && m.Kd == 10 && m.q == 0 && m.dq == 0 && m.tau == 0,
                "Stop/fault did not prepare passive damping");
}
void sameCommand(const LowlevelCmd &actual, const LowlevelCmd &expected, const char *message)
{
    for (int j = 0; j < 29; ++j) {
        const auto &a = actual.motorCmd[j], &e = expected.motorCmd[j];
        require(a.mode == e.mode && std::abs(a.q - e.q) < 1e-5 &&
                std::abs(a.dq - e.dq) < 1e-5 && std::abs(a.tau - e.tau) < 1e-5 &&
                std::abs(a.Kp - e.Kp) < 1e-4 && std::abs(a.Kd - e.Kd) < 1e-5, message);
    }
}
int main()
{
    try {
        CtrlComponents ctrl(nullptr);
        ctrl.dt = 0.02;
        State_Loco loco(&ctrl);
        State_MJAMP_RECOVERY recovery(&ctrl);
        State_LocoRecovery2 state(&ctrl, loco, recovery);
        CtrlComponents reference(nullptr);
        reference.dt = ctrl.dt;
        State_MJAMP_RECOVERY standaloneRecovery(&reference);
        auto referenceStart = [&] {
            *reference.lowState = *ctrl.lowState;
            standaloneRecovery.enterAutomatic();
            require(!standaloneRecovery.failed(), "Reference Recovery failed to activate");
            return *reference.lowCmd;
        };
        auto referenceStep = [&] {
            *reference.lowState = *ctrl.lowState;
            standaloneRecovery.run();
            require(!standaloneRecovery.failed(), "Reference Recovery inference failed");
            return *reference.lowCmd;
        };
        auto step = [&] {
            state.run();
            require(state.checkChange() == FSMStateName::LOCO_RECOVERY2,
                    "Unexpected exit from second composite task");
        };
        auto restart = [&] { state.exit(); pose(ctrl); state.enter(); };
        auto fall = [&] {
            pose(ctrl, 1.1); step();
            require(state.phase() == Phase::PROTECT_WAIT, "Hard fall did not enter protection");
            protection(*ctrl.lowCmd);
        };
        auto activate = [&] {
            command(ctrl, UserCommand::NONE); step();
            command(ctrl, UserCommand::R1); step();
            require(state.phase() == Phase::RECOVERY, "Released then freshly pressed R1 did not start Recovery");
        };
        pose(ctrl); state.enter(); step();
        require(state.phase() == Phase::LOCO && ctrl.lowCmd->motorCmd[0].Kp == 200,
                "Standing entry did not select normal Loco");

        // Ordinary wobbling must retain locomotion; this task triggers after a
        // clear fall, including a stationary fallen pose with no outward gyro.
        pose(ctrl, 0.55); ctrl.lowState->imu.gyroscope[1] = 3;
        for (int i = 0; i < 8; ++i) step();
        require(state.phase() == Phase::LOCO, "Old early-recovery tilt was reused for protection");
        pose(ctrl, 0.99); step();
        require(state.phase() == Phase::LOCO, "Non-hard fall ignored the protection debounce");
        pose(ctrl, 0.55); step();
        pose(ctrl, 0.99); step();
        require(state.phase() == Phase::LOCO, "Interrupted fall did not reset the debounce");
        step();
        require(state.phase() == Phase::PROTECT_WAIT, "Stationary fallen feedback did not trigger protection");
        protection(*ctrl.lowCmd);
        for (bool roll : {false, true}) {
            for (float sign : {-1.0f, 1.0f}) {
                restart(); pose(ctrl, sign * 0.99f, roll); step(); step();
                require(state.phase() == Phase::PROTECT_WAIT, "Directional fall was missed");
                protection(*ctrl.lowCmd);
                restart(); pose(ctrl, sign * 1.1f, roll); step();
                require(state.phase() == Phase::PROTECT_WAIT, "Hard fall was delayed by the debounce");
            }
        }

        // More than the old 15-second timeout cannot start either active mode.
        // Moving the robot or pressing mode buttons while waiting cannot bypass R1.
        for (int frame = 0; frame < 810; ++frame) {
            pose(ctrl, frame % 2 ? 0.1 : 1.3);
            ctrl.lowState->motorState[0].q += 0.1f * (frame % 5);
            ctrl.lowState->motorState[5].dq = 0.4;
            ctrl.lowState->userValue.ly = 0.8;
            const UserCommand ignored[] = {UserCommand::R2_X, UserCommand::R2_Y,
                UserCommand::R2_A, UserCommand::R2_B, UserCommand::START,
                UserCommand::R1_UP, UserCommand::R1_LEFT, UserCommand::R1_RIGHT};
            command(ctrl, ignored[frame % 8]); step();
            require(state.phase() == Phase::PROTECT_WAIT, "Waiting autonomously resumed an active policy");
            protection(*ctrl.lowCmd);
        }

        // Holding R1 before a fall is not permission to start get-up. Both raw
        // held buttons and the emitted command must report release before arming.
        restart(); pose(ctrl, 1.1); command(ctrl, UserCommand::R1); step();
        for (int i = 0; i < 12; ++i) {
            step(); protection(*ctrl.lowCmd);
            require(state.phase() == Phase::PROTECT_WAIT, "Pre-fall held R1 started Recovery");
        }
        ctrl.lowState->userCmd = UserCommand::NONE;
        ctrl.lowState->heldUserCmd = UserCommand::R1;
        step(); command(ctrl, UserCommand::R1); step();
        require(state.phase() == Phase::PROTECT_WAIT, "Filtered held R1 was mistaken for release");
        ctrl.lowState->userCmd = UserCommand::NONE;
        ctrl.lowState->heldUserCmd = UserCommand::R2_X;
        step(); command(ctrl, UserCommand::R1); step();
        require(state.phase() == Phase::PROTECT_WAIT, "Filtered held recovery key armed R1");
        command(ctrl, UserCommand::NONE); step();
        ctrl.lowState->userCmd = UserCommand::R1;
        ctrl.lowState->heldUserCmd = UserCommand::NONE;
        step(); protection(*ctrl.lowCmd);
        require(state.phase() == Phase::PROTECT_WAIT, "Stale logical R1 without a physical button started get-up");
        command(ctrl, UserCommand::NONE); step();
        for (auto button : {UserCommand::R1_UP, UserCommand::R1_LEFT,
                           UserCommand::R1_RIGHT, UserCommand::R1_OTHER}) {
            command(ctrl, button); step();
            require(state.phase() == Phase::PROTECT_WAIT, "R1 plus a direction started get-up");
            protection(*ctrl.lowCmd);
        }
        command(ctrl, UserCommand::R1); step();
        require(state.phase() == Phase::PROTECT_WAIT, "Releasing only a direction while R1 stayed held started get-up");
        protection(*ctrl.lowCmd);
        command(ctrl, UserCommand::NONE); step();

        // The body may be repositioned while unloaded. The first Recovery action
        // must match an independent policy started from the actual R1 sample.
        pose(ctrl, 1.25, true);
        ctrl.lowState->motorState[0].q = -0.9;
        ctrl.lowState->motorState[3].q = 1.6;
        ctrl.lowState->motorState[17].q = 0.35;
        ctrl.lowState->userValue.ly = 0.9;
        command(ctrl, UserCommand::R1); step();
        require(state.phase() == Phase::RECOVERY, "Fresh R1 did not activate from the current pose");
        sameCommand(*ctrl.lowCmd, referenceStart(), "R1 activation used stale pose/history or nonzero velocity");
        for (int frame = 0; frame < 8; ++frame) {
            ctrl.lowState->motorState[0].q += 0.012f;
            ctrl.lowState->motorState[3].dq = 0.05;
            step();
            sameCommand(*ctrl.lowCmd, referenceStep(), "Held R1 reinitialized the active Recovery policy");
        }
        require(state.phase() == Phase::RECOVERY, "Lying Recovery feedback restarted protection");

        // Recovery is allowed to pass through fallen/kneeling poses and retains
        // support even if it has not reached a standing stance after 15 seconds.
        pose(ctrl, 1.1);
        for (int i = 0; i < 802; ++i) {
            step();
            require(state.phase() == Phase::RECOVERY, "Slow get-up was reprotected or timed out");
            for (int j = 0; j < 12; ++j)
                require(ctrl.lowCmd->motorCmd[j].Kp > 0, "Slow get-up withdrew joint support");
        }
        pose(ctrl);
        ctrl.lowState->motorState[3].q = ctrl.lowState->motorState[9].q = 1.8;
        for (int i = 0; i < 45; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Kneeling counted as a standing stance");
        pose(ctrl); ctrl.lowState->motorState[0].dq = 1;
        for (int i = 0; i < 45; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Moving legs counted as a standing stance");
        pose(ctrl); ctrl.lowState->imu.gyroscope[2] = 1;
        for (int i = 0; i < 45; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Angular velocity standing gate was skipped");
        pose(ctrl);
        for (int i = 0; i < 25; ++i) step();
        pose(ctrl, 0.3); step(); pose(ctrl);
        for (int i = 0; i < 39; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Interrupted stance did not reset the standing timer");
        step();
        require(state.phase() == Phase::RETURNING, "Continuous stable standing did not start the return");
        const auto returnStart = *ctrl.lowCmd;
        step();
        require(ctrl.lowCmd->motorCmd[0].Kp > returnStart.motorCmd[0].Kp &&
                ctrl.lowCmd->motorCmd[0].Kp < 200, "Return gains did not transition smoothly");
        fall();
        command(ctrl, UserCommand::R1); step();
        require(state.phase() == Phase::PROTECT_WAIT, "Repeated fall reused an earlier R1 authorization");
        activate(); pose(ctrl);
        for (int i = 0; i < 49; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Minimum Recovery time was bypassed");
        step();
        require(state.phase() == Phase::RETURNING, "Second authorized Recovery failed to return");
        for (int i = 0; i < 15; ++i) step();
        require(state.phase() == Phase::LOCO && ctrl.lowCmd->motorCmd[0].Kp == 200,
                "Second return blend did not finish");
        fall();

        state.exit(); pose(ctrl, 0.3); command(ctrl, UserCommand::R1); state.enter();
        require(state.phase() == Phase::PROTECT_WAIT, "Held R1 at non-standing entry activated Recovery");
        for (int i = 0; i < 4; ++i) { step(); protection(*ctrl.lowCmd); }
        require(state.phase() == Phase::PROTECT_WAIT, "Entry-sample deduplication counted as R1 release");
        activate();

        // Non-standing task entry also waits instead of inferring get-up. Different
        // wait lengths and changed waiting poses must not leak into the new episode.
        auto episode = [&](bool heldStick, int waitFrames) {
            state.exit(); pose(ctrl, 0.3); state.enter();
            require(state.phase() == Phase::PROTECT_WAIT, "Non-standing entry activated a policy");
            protection(*ctrl.lowCmd);
            step(); // Match the outer FSM's enter()+run() on the entry sample.
            for (int i = 0; i < waitFrames; ++i) {
                pose(ctrl, 0.7f + 0.03f * (i % 7), i % 2);
                ctrl.lowState->motorState[0].q = -0.8f + 0.02f * i;
                ctrl.lowState->userValue.ly = heldStick ? 0.8 : 0;
                step(); protection(*ctrl.lowCmd);
            }
            pose(ctrl, 1.1);
            ctrl.lowState->userValue.ly = heldStick ? 0.8 : 0;
            command(ctrl, UserCommand::R1); step();
            require(state.phase() == Phase::RECOVERY, "Episode R1 did not activate");
            std::vector<LowlevelCmd> trace{*ctrl.lowCmd};
            for (int i = 0; i < 85; ++i) {
                if (i == 5) {
                    pose(ctrl);
                    ctrl.lowState->userValue.ly = heldStick ? 0.8 : 0;
                }
                step(); trace.push_back(*ctrl.lowCmd);
            }
            require(state.phase() == Phase::LOCO, "Episode did not return to Loco");
            return trace;
        };
        const auto zero = episode(false, 1);
        const auto held = episode(true, 23);
        for (size_t t = 0; t < zero.size(); ++t)
            sameCommand(held[t], zero[t], "Waiting history or held stick affected get-up/return before recentering");
        ctrl.lowState->userValue.setZero(); step();
        ctrl.lowState->userValue.ly = 0.8; step();
        require(state.phase() == Phase::LOCO, "Stick recentering did not retain Loco");

        // Stop, shutdown, and invalid feedback take priority in every phase.
        for (int phase = 0; phase < 4; ++phase) {
            auto setupPhase = [&] {
                restart();
                if (phase > 0) fall();
                if (phase > 1) activate();
                if (phase > 2) {
                    pose(ctrl); for (int i = 0; i < 50; ++i) step();
                    require(state.phase() == Phase::RETURNING, "Interruption setup missed RETURNING");
                }
            };
            setupPhase(); command(ctrl, UserCommand::L2_B); state.run(); damping(*ctrl.lowCmd);
            require(state.checkChange() == FSMStateName::PASSIVE, "Stop lost priority");
            setupPhase(); command(ctrl, UserCommand::SELECT); state.run(); damping(*ctrl.lowCmd);
            bool exited = false;
            try { state.checkChange(); } catch (const std::runtime_error &) { exited = true; }
            require(exited, "SELECT did not exit from every phase");
            setupPhase(); ctrl.lowState->motorState[0].dq = std::numeric_limits<float>::quiet_NaN();
            state.run(); damping(*ctrl.lowCmd);
            require(state.checkChange() == FSMStateName::PASSIVE, "Invalid joint feedback did not stop");
            pose(ctrl); state.run(); damping(*ctrl.lowCmd);
            require(state.checkChange() == FSMStateName::PASSIVE, "Fault did not remain latched");
        }
        restart(); ctrl.lowState->imu.quaternion[0] = 0;
        state.run(); damping(*ctrl.lowCmd);
        restart(); ctrl.dt = 0.01;
        state.run(); damping(*ctrl.lowCmd); ctrl.dt = 0.02;

        // Borrowed policies must retain their original independent operation.
        state.exit(); pose(ctrl); ctrl.lowState->userValue.ly = 0.7;
        loco.enter(); loco.run(); const LowlevelCmd afterComposite = *ctrl.lowCmd;
        State_Loco baseline(&ctrl); baseline.enter(); baseline.run();
        sameCommand(*ctrl.lowCmd, afterComposite, "Second task leaked gain/history into ordinary Loco");
        pose(ctrl); recovery.enter(); recovery.run(); damping(*ctrl.lowCmd);
        std::cout << "PASS: fallen protection/debounce, indefinite zero output, exact released R1, "
                     "fresh-pose get-up, held R1, stance gates, smooth repeated return, waiting "
                     "history/stick isolation, stop/fault/shutdown and independent policies "
                     "(real ONNX, no DDS)\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
