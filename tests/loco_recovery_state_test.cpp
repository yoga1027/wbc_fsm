#include "FSM/State_LocoRecovery.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using Phase = State_LocoRecovery::Phase;
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
void damping(CtrlComponents &ctrl)
{
    for (const auto &c : ctrl.lowCmd->motorCmd)
        require(c.Kp == 0 && c.Kd == 10 && c.q == 0 && c.dq == 0 && c.tau == 0,
                "Stop/fault left active commands");
}
void sameCommand(const LowlevelCmd &actual, const LowlevelCmd &expected, const char *message)
{
    for (int j = 0; j < 29; ++j) {
        const auto &a = actual.motorCmd[j];
        const auto &e = expected.motorCmd[j];
        require(a.mode == e.mode && std::abs(a.q - e.q) < 1e-5 &&
                std::abs(a.dq - e.dq) < 1e-5 && std::abs(a.tau - e.tau) < 1e-5 &&
                std::abs(a.Kp - e.Kp) < 1e-4 && std::abs(a.Kd - e.Kd) < 1e-5, message);
    }
}
// Derive observed progress from a gain, then check that all command fields use
// that progress against the current, independently inferred Recovery action.
double blendProgress(const LowlevelCmd &actual, const LowlevelCmd &from,
                     const LowlevelCmd &target)
{
    const double alpha = (actual.motorCmd[0].Kp - from.motorCmd[0].Kp) /
                         (target.motorCmd[0].Kp - from.motorCmd[0].Kp);
    LowlevelCmd expected = target;
    for (int j = 0; j < 29; ++j) {
        auto &e = expected.motorCmd[j];
        const auto &f = from.motorCmd[j];
        const auto &t = target.motorCmd[j];
        e.q = (1 - alpha) * f.q + alpha * t.q;
        e.dq = (1 - alpha) * f.dq + alpha * t.dq;
        e.tau = (1 - alpha) * f.tau + alpha * t.tau;
        e.Kp = (1 - alpha) * f.Kp + alpha * t.Kp;
        e.Kd = (1 - alpha) * f.Kd + alpha * t.Kd;
    }
    sameCommand(actual, expected, "Takeover did not blend all fields toward the latest Recovery action");
    return alpha;
}
int main()
{
    try {
        CtrlComponents ctrl(nullptr);
        ctrl.dt = 0.02;
        State_Loco loco(&ctrl);
        State_MJAMP_RECOVERY recovery(&ctrl);
        State_LocoRecovery state(&ctrl, loco, recovery);
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
            require(state.checkChange() == FSMStateName::LOCO_RECOVERY, "Unexpected composite exit");
        };
        auto restart = [&] { state.exit(); pose(ctrl); state.enter(); };
        pose(ctrl);
        state.enter();
        for (int i = 0; i < 5; ++i) step();
        require(state.phase() == Phase::LOCO, "Standing entry did not select Loco");

        // Inward correction and pure yaw must not trigger the early branch.
        pose(ctrl, 0.55);
        ctrl.lowState->imu.gyroscope[1] = -1;
        for (int i = 0; i < 5; ++i) step();
        require(state.phase() == Phase::LOCO, "Inward correction triggered recovery");
        ctrl.lowState->imu.gyroscope[1] = 0;
        ctrl.lowState->imu.gyroscope[2] = 3;
        for (int i = 0; i < 5; ++i) step();
        require(state.phase() == Phase::LOCO, "Yaw triggered recovery");

        // Two consecutive outward samples; interruption resets debounce.
        ctrl.lowState->imu.gyroscope[2] = 0;
        ctrl.lowState->imu.gyroscope[1] = 1;
        step();
        require(state.phase() == Phase::LOCO, "Early trigger ignored debounce");
        ctrl.lowState->imu.gyroscope[1] = -1;
        step();
        ctrl.lowState->imu.gyroscope[1] = 1;
        step();
        require(state.phase() == Phase::LOCO, "Debounce was not reset");
        step();
        require(state.phase() == Phase::RECOVERY && ctrl.lowCmd->motorCmd[0].Kp < 200 &&
                ctrl.lowCmd->motorCmd[0].Kp > 100,
                "Non-emergency takeover did not begin a supported command handoff in the trigger cycle");

        // A mild fall gets a short, progressive handoff. Recovery keeps inferring
        // every frame instead of replaying the first target for the whole blend.
        restart();
        pose(ctrl, 0.55);
        ctrl.lowState->imu.gyroscope[1] = 1;
        step();
        // Nonzero velocity/feedforward also have to reach Recovery's zero values.
        ctrl.lowCmd->motorCmd[0].dq = 0.3;
        ctrl.lowCmd->motorCmd[0].tau = 2;
        const LowlevelCmd takeoverFrom = *ctrl.lowCmd;
        step();
        const LowlevelCmd firstRecovery = referenceStart();
        double progress = blendProgress(*ctrl.lowCmd, takeoverFrom, firstRecovery);
        require(progress > 0 && progress < 0.5,
                "Mild takeover did not preserve the previous active command on its first frame");
        bool recoveryTargetChanged = false;
        for (int frame = 1; frame <= 3; ++frame) {
            pose(ctrl, 0.55f - frame * 0.01f);
            ctrl.lowState->imu.gyroscope[1] = 0.6;
            ctrl.lowState->motorState[0].q += frame * 0.02f;
            ctrl.lowState->motorState[0].dq = 0.1;
            step();
            const auto raw = referenceStep();
            for (int j = 0; j < 29; ++j)
                recoveryTargetChanged |= std::abs(raw.motorCmd[j].q - firstRecovery.motorCmd[j].q) > 1e-4;
            const double nextProgress = blendProgress(*ctrl.lowCmd, takeoverFrom, raw);
            require(nextProgress > progress && nextProgress <= 1.00001,
                    "Takeover blend did not progress monotonically");
            progress = nextProgress;
        }
        require(recoveryTargetChanged, "Reference Recovery target did not vary in live-target regression");
        require(progress > 0.99999, "Short takeover blend did not finish within four control samples");
        sameCommand(*ctrl.lowCmd, *reference.lowCmd, "Completed takeover retained an old command");

        // The predicted time remaining before the hard tilt shortens a handoff.
        restart(); pose(ctrl, 0.67); ctrl.lowState->imu.gyroscope[1] = 1;
        step();
        const LowlevelCmd shortFrom = *ctrl.lowCmd;
        step();
        progress = blendProgress(*ctrl.lowCmd, shortFrom, referenceStart());
        require(progress > 0.5 && progress < 1,
                "Near-hard-tilt takeover did not shorten the handoff");
        step();
        sameCommand(*ctrl.lowCmd, referenceStep(), "Shortened takeover did not promptly finish");

        // Rising risk below the emergency thresholds must shorten an in-progress
        // handoff; a later safer sample must not extend that shortened deadline.
        restart(); pose(ctrl, 0.55); ctrl.lowState->imu.gyroscope[1] = 1;
        step();
        const LowlevelCmd dynamicFrom = *ctrl.lowCmd;
        step();
        progress = blendProgress(*ctrl.lowCmd, dynamicFrom, referenceStart());
        require(progress > 0 && progress < 0.5,
                "Dynamic-shortening setup did not begin a mild takeover");
        pose(ctrl, 0.67); ctrl.lowState->imu.gyroscope[1] = 1;
        step();
        const double shortenedProgress = blendProgress(*ctrl.lowCmd, dynamicFrom, referenceStep());
        require(shortenedProgress > progress && shortenedProgress > 0.5 && shortenedProgress < 1,
                "Rising non-emergency risk did not shorten the running handoff");
        pose(ctrl, 0.55); ctrl.lowState->imu.gyroscope[1] = 0.6;
        step();
        progress = blendProgress(*ctrl.lowCmd, dynamicFrom, referenceStep());
        require(progress > shortenedProgress && progress > 0.99999,
                "Falling risk extended the shortened handoff beyond three control samples");
        sameCommand(*ctrl.lowCmd, *reference.lowCmd,
                    "Dynamically shortened takeover retained an old command");

        // Emergency cases preserve immediate Recovery authority on the trigger cycle.
        for (bool fastRate : {false, true}) {
            restart(); pose(ctrl, fastRate ? 0.55 : 0.8);
            ctrl.lowState->imu.gyroscope[1] = fastRate ? 3 : 0;
            step();
            if (fastRate) step(); // Early tilt still uses the existing debounce.
            require(state.phase() == Phase::RECOVERY, "Emergency takeover did not enter Recovery");
            sameCommand(*ctrl.lowCmd, referenceStart(), "Emergency takeover was delayed by blending");
        }
        restart(); pose(ctrl, 0.70); ctrl.lowState->imu.gyroscope[1] = 1;
        step(); step();
        sameCommand(*ctrl.lowCmd, referenceStart(),
                    "A fall with less than one safe control sample was still blended");

        // A dangerous new sample also cuts short an already running handoff.
        for (bool fastRate : {false, true}) {
            restart(); pose(ctrl, 0.55); ctrl.lowState->imu.gyroscope[1] = 1;
            step(); step(); referenceStart();
            require(ctrl.lowCmd->motorCmd[0].Kp > 100, "Upgrade regression never entered a blend");
            pose(ctrl, fastRate ? 0.56 : 0.8);
            ctrl.lowState->imu.gyroscope[1] = fastRate ? 3 : 0;
            step();
            sameCommand(*ctrl.lowCmd, referenceStep(), "Worsening danger did not cancel takeover blend");
        }

        // A repeat fall during return starts from the most recently published
        // intermediate command, rather than the first episode's Loco command.
        restart(); pose(ctrl, 0.8); step(); pose(ctrl);
        for (int i = 0; i < 50; ++i) step();
        require(state.phase() == Phase::RETURNING, "Repeat-handoff setup did not enter return blend");
        for (int i = 0; i < 3; ++i) step();
        pose(ctrl, 0.55); ctrl.lowState->imu.gyroscope[1] = 1;
        step();
        const LowlevelCmd repeatFrom = *ctrl.lowCmd;
        require(repeatFrom.motorCmd[0].Kp < 200, "Repeat-handoff setup already reached full Loco");
        step();
        progress = blendProgress(*ctrl.lowCmd, repeatFrom, referenceStart());
        require(state.phase() == Phase::RECOVERY && progress > 0 && progress < 0.5,
                "Repeat handoff did not use the current intermediate command");

        // Entry from an unsupported posture must activate Recovery directly,
        // without slowly ramping stiffness upward from passive damping.
        state.exit(); pose(ctrl, 0.3);
        for (auto &cmd : ctrl.lowCmd->motorCmd) {
            cmd.mode = 10; cmd.q = cmd.dq = cmd.tau = cmd.Kp = 0; cmd.Kd = 10;
        }
        state.enter();
        require(state.phase() == Phase::RECOVERY, "Non-standing entry did not choose Recovery");
        const LowlevelCmd directEntry = referenceStart();
        sameCommand(*ctrl.lowCmd, directEntry, "Non-standing entry ramped up from damping");
        step();
        sameCommand(*ctrl.lowCmd, directEntry, "Entry action was unexpectedly inferred twice");

        // Stop and invalid feedback must interrupt even the first blending frames.
        for (int interruption = 0; interruption < 3; ++interruption) {
            restart(); pose(ctrl, 0.55); ctrl.lowState->imu.gyroscope[1] = 1;
            step(); step();
            require(ctrl.lowCmd->motorCmd[0].Kp > 100, "Interruption setup never entered a blend");
            if (interruption == 0) ctrl.lowState->userCmd = UserCommand::L2_B;
            else if (interruption == 1) ctrl.lowState->userCmd = UserCommand::SELECT;
            else ctrl.lowState->motorState[0].dq = std::numeric_limits<float>::quiet_NaN();
            state.run(); damping(ctrl);
            if (interruption == 1) {
                bool exited = false;
                try { state.checkChange(); } catch (const std::runtime_error &) { exited = true; }
                require(exited, "SELECT lost priority during takeover blend");
            } else require(state.checkChange() == FSMStateName::PASSIVE,
                           "Stop/fault did not leave takeover blend");
        }

        for (bool roll : {false, true}) {
            for (float sign : {-1.0f, 1.0f}) {
                restart();
                pose(ctrl, sign * 0.55f, roll);
                ctrl.lowState->imu.gyroscope[roll ? 0 : 1] = sign;
                step(); step();
                require(state.phase() == Phase::RECOVERY, "Directional early trigger missing");
            }
        }
        restart();
        pose(ctrl, 0.8);
        step();
        require(state.phase() == Phase::RECOVERY, "Hard tilt did not take over immediately");

        // Upright torso while sitting/kneeling, moving legs or rotating is not standing.
        pose(ctrl);
        ctrl.lowState->motorState[3].q = ctrl.lowState->motorState[9].q = 1.8;
        ctrl.lowState->userCmd = UserCommand::R2_A;
        for (int i = 0; i < 55; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Kneeling or held mode key bypassed standing gate");
        pose(ctrl);
        ctrl.lowState->motorState[0].q = ctrl.lowState->motorState[6].q = -1.3;
        ctrl.lowState->motorState[3].q = ctrl.lowState->motorState[9].q = 1.4;
        for (int i = 0; i < 55; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Upright sitting pose counted as standing");
        pose(ctrl);
        ctrl.lowState->motorState[0].dq = 1;
        for (int i = 0; i < 45; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Moving legs counted as standing");
        pose(ctrl);
        ctrl.lowState->imu.gyroscope[2] = 1;
        for (int i = 0; i < 45; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Angular speed gate missing");
        pose(ctrl);
        for (int i = 0; i < 25; ++i) step();
        pose(ctrl, 0.3); step();
        pose(ctrl);
        for (int i = 0; i < 39; ++i) step();
        require(state.phase() == Phase::RECOVERY, "Stability timer did not reset");
        step();
        require(state.phase() == Phase::RETURNING, "Stable stance did not return automatically");
        const float recoveryKp = ctrl.lowCmd->motorCmd[0].Kp;
        step();
        require(ctrl.lowCmd->motorCmd[0].Kp > recoveryKp &&
                ctrl.lowCmd->motorCmd[0].Kp < 200, "PD gains were not blended");
        pose(ctrl, 0.8); step();
        require(state.phase() == Phase::RECOVERY, "Return blend masked a second fall");
        pose(ctrl);
        for (int i = 0; i < 50; ++i) step();
        require(state.phase() == Phase::RETURNING, "Repeated recovery failed");
        for (int i = 0; i < 15; ++i) step();
        require(state.phase() == Phase::LOCO && ctrl.lowCmd->motorCmd[0].Kp == 200,
                "Return blend did not complete");

        // Held joystick must not affect recovery, return, or Loco before recentering.
        // Repeated episodes also verify that recurrent/history state was cleared.
        auto episode = [&](bool held) {
            state.exit(); pose(ctrl, 0.8);
            ctrl.lowState->userValue.ly = held ? 0.8 : 0;
            state.enter();
            std::vector<LowlevelCmd> trace;
            for (int i = 0; i < 85; ++i) {
                if (i == 5) {
                    pose(ctrl);
                    ctrl.lowState->userValue.ly = held ? 0.8 : 0;
                }
                step(); trace.push_back(*ctrl.lowCmd);
            }
            require(state.phase() == Phase::LOCO, "Episode did not return to Loco");
            return trace;
        };
        const auto zero = episode(false);
        const auto held = episode(true);
        for (size_t t = 0; t < zero.size(); ++t)
            for (int j = 0; j < 29; ++j)
                require(std::abs(zero[t].motorCmd[j].q - held[t].motorCmd[j].q) < 1e-6,
                        "Held stick affected auto recovery/return or policy state leaked");
        ctrl.lowState->userValue.setZero(); step();
        ctrl.lowState->userValue.ly = 0.8;
        step();
        require(state.phase() == Phase::LOCO, "Recentering failed to resume Loco");

        // Faults are latched until explicit state re-entry.
        ctrl.lowState->motorState[0].q = std::numeric_limits<float>::quiet_NaN();
        state.run(); damping(ctrl);
        require(state.checkChange() == FSMStateName::PASSIVE, "NaN did not stop composite");
        pose(ctrl); state.run(); damping(ctrl);
        restart();
        ctrl.lowState->imu.quaternion[0] = 0;
        state.run(); damping(ctrl);
        restart(); ctrl.dt = 0.01;
        state.run(); damping(ctrl); ctrl.dt = 0.02;

        // Slow recoveries must keep active control past the warning deadline.
        // Once the feedback actually satisfies the stance gate they still return automatically.
        restart(); pose(ctrl, 0.8); step();
        for (int i = 0; i < 802; ++i) {
            step();
            require(state.phase() == Phase::RECOVERY, "Recovery deadline ended active recovery");
            for (int joint = 0; joint < 12; ++joint)
                require(ctrl.lowCmd->motorCmd[joint].Kp > 0,
                        "Recovery deadline left a leg without active stiffness");
        }
        pose(ctrl);
        for (int i = 0; i < 40; ++i) step();
        require(state.phase() == Phase::RETURNING, "Recovery could not return after warning deadline");
        for (int i = 0; i < 15; ++i) step();
        require(state.phase() == Phase::LOCO, "Recovery return after warning did not complete");

        // The Recovery policy can stand on straight legs; positive knee flexion is not required.
        restart(); pose(ctrl, 0.8); step();
        pose(ctrl);
        for (int offset : {0, 6}) {
            ctrl.lowState->motorState[offset].q = -0.08;
            ctrl.lowState->motorState[offset + 3].q = 0;
            ctrl.lowState->motorState[offset + 4].q = 0.12;
        }
        for (int i = 0; i < 50; ++i) step();
        require(state.phase() == Phase::RETURNING, "Straight legs were rejected as a stable stance");
        for (int i = 0; i < 15; ++i) step();
        require(state.phase() == Phase::LOCO, "Straight-leg recovery did not return to Loco");

        // A recovered stance need not match the fixed stand's narrow hip ranges.
        // These hip values were observed on an upright, nearly stationary recovery pose.
        restart(); pose(ctrl, 0.8); step();
        pose(ctrl);
        for (int offset : {0, 6}) {
            ctrl.lowState->motorState[offset].q = 0.3;
            ctrl.lowState->motorState[offset + 1].q = -0.47;
            ctrl.lowState->motorState[offset + 2].q = -1.07;
        }
        for (int i = 0; i < 50; ++i) step();
        require(state.phase() == Phase::RETURNING, "Recovered hip stance was rejected as unstable");
        for (int i = 0; i < 15; ++i) step();
        require(state.phase() == Phase::LOCO, "Recovered hip stance did not return to Loco");
        for (bool inRecovery : {false, true}) {
            restart();
            if (inRecovery) { pose(ctrl, 0.8); step(); }
            ctrl.lowState->userCmd = UserCommand::L2_B;
            state.run(); damping(ctrl);
            require(state.checkChange() == FSMStateName::PASSIVE, "Manual stop lost priority");
            ctrl.lowState->userCmd = UserCommand::SELECT;
            state.run(); damping(ctrl);
            bool exited = false;
            try { state.checkChange(); } catch (const std::runtime_error &) { exited = true; }
            require(exited, "SELECT did not exit");
        }
        state.exit();
        pose(ctrl);
        ctrl.lowState->userValue.ly = 0.7;
        loco.enter(); loco.run();
        const LowlevelCmd afterComposite = *ctrl.lowCmd;
        State_Loco baseline(&ctrl);
        baseline.enter(); baseline.run();
        for (int j = 0; j < 29; ++j)
            require(std::abs(afterComposite.motorCmd[j].q - ctrl.lowCmd->motorCmd[j].q) < 1e-6,
                    "Composite leaked command gain/history into ordinary Loco");
        // Borrowing must not change the independent recovery's manual start.
        pose(ctrl); recovery.enter(); recovery.run(); damping(ctrl);
        std::cout << "PASS: automatic takeover, direction/debounce, stance gates, repeat recovery, "
                     "live-target adaptive takeover/return blends, held commands, faults, "
                     "slow/straight-leg recovery and stops (real ONNX, no DDS)\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
