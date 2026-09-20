#include "FSM/State_MJAmpRecovery.h"
#include "FSM/State_MJAmp.h"
#include "FSM/State_Loco.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "interface/UserCommandMapping.h"
#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

using json = nlohmann::json;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

void pose(CtrlComponents &ctrl, float tilt = 0.0f)
{
    auto &state = *ctrl.lowState;
    state.userCmd = UserCommand::NONE;
    state.heldUserCmd = UserCommand::NONE;
    state.userValue.setZero();
    state.imu.quaternion[0] = std::cos(tilt / 2);
    state.imu.quaternion[1] = 0;
    state.imu.quaternion[2] = std::sin(tilt / 2);
    state.imu.quaternion[3] = 0;
    for (int i = 0; i < 3; ++i) state.imu.gyroscope[i] = 0.04f * (i + 1);
    for (int i = 0; i < 29; ++i) {
        state.motorState[i].q = 0.1f * std::sin(float(i));
        state.motorState[i].dq = 0.03f * std::cos(float(i));
    }
}

void requireDamping(CtrlComponents &ctrl)
{
    for (const auto &cmd : ctrl.lowCmd->motorCmd)
        require(cmd.Kp == 0 && cmd.Kd == 10 && cmd.tau == 0 && cmd.dq == 0 && cmd.q == 0,
                "Fault/stop must overwrite every joint with damping");
}

// Observe a real release while waiting, then leave a fresh start for the next run().
void requestStart(State_MJAMP_RECOVERY &recovery, CtrlComponents &ctrl)
{
    ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = UserCommand::NONE;
    recovery.run();
    requireDamping(ctrl);
    require(recovery.checkChange() == FSMStateName::MJAMP_RECOVERY, "Waiting unexpectedly exited");
    ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = UserCommand::R2_Y;
}

std::array<float, 29> targets(CtrlComponents &ctrl)
{
    std::array<float, 29> result{};
    for (int i = 0; i < 29; ++i) result[i] = ctrl.lowCmd->motorCmd[i].q;
    return result;
}

void testButtons()
{
    unitree::common::Gamepad pad;
    RecoveryEntryLatch latch;
    require(userCommandFromGamepad(pad) == UserCommand::NONE, "Released buttons must clear command");
    pad.R2.pressed = pad.X.pressed = true;
    require(latch.sample(userCommandFromGamepad(pad)) == UserCommand::R2_X, "Recovery combination");
    pad.Y.pressed = true;
    require(userCommandFromGamepad(pad) == UserCommand::R2_X, "Entry must win over simultaneous start");
    require(latch.sample(userCommandFromGamepad(pad)) == UserCommand::NONE, "Held key must not retrigger");
    pad.L2.pressed = pad.B.pressed = true;
    require(userCommandFromGamepad(pad) == UserCommand::L2_B, "Damping must override recovery");
    pad.select.pressed = true;
    require(userCommandFromGamepad(pad) == UserCommand::SELECT, "SELECT has highest priority");
    pad = unitree::common::Gamepad{};
    latch.sample(userCommandFromGamepad(pad));
    pad.R2.pressed = pad.X.pressed = true;
    require(latch.sample(userCommandFromGamepad(pad)) == UserCommand::R2_X, "Release must rearm entry");
    pad.X.pressed = false;
    pad.A.pressed = true;
    require(userCommandFromGamepad(pad) == UserCommand::R2_A, "Original MJAMP combination");
    pad.A.pressed = false;
    pad.B.pressed = true;
    require(userCommandFromGamepad(pad) == UserCommand::R2_B, "Original LOCO combination");
    pad.B.pressed = false;
    pad.Y.pressed = true;
    require(userCommandFromGamepad(pad) == UserCommand::R2_Y, "Recovery start combination");
    pad.L2.pressed = pad.B.pressed = true;
    require(userCommandFromGamepad(pad) == UserCommand::L2_B, "Damping must override start");
    pad.select.pressed = true;
    require(userCommandFromGamepad(pad) == UserCommand::SELECT, "SELECT must override start");
}

int main(int argc, char **argv)
{
    try {
        testButtons();
        CtrlComponents ctrl(nullptr); // No IOSDK construction, DDS connection or command publishing.
        ctrl.dt = 0.02;
        pose(ctrl);
        State_MJAMP_RECOVERY recovery(&ctrl);
        State_MJAMP original(&ctrl);
        State_Loco loco(&ctrl);
        State_FixedStand fixed(&ctrl);
        State_Passive passive(&ctrl);
        ctrl.lowState->userCmd = UserCommand::R2_X;
        for (FSMState *state : std::array<FSMState *, 4>{&original, &loco, &fixed, &passive})
            require(state->checkChange() == FSMStateName::MJAMP_RECOVERY, "Missing recovery entry");

        // Waiting remains damping across time, joystick motion, and other mode requests.
        pose(ctrl);
        recovery.enter();
        requireDamping(ctrl);
        for (int i = 0; i < 100; ++i) {
            ctrl.lowState->userValue.ly = 0.8f;
            ctrl.lowState->motorState[3].q += 0.01f;
            const std::array<UserCommand, 5> ignored{UserCommand::R2_A, UserCommand::R2_B,
                UserCommand::R2_UP, UserCommand::R1_UP, UserCommand::START};
            ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = ignored[i % ignored.size()];
            recovery.run();
            requireDamping(ctrl);
            require(recovery.checkChange() == FSMStateName::MJAMP_RECOVERY, "Waiting bypassed explicit start");
        }
        requestStart(recovery, ctrl);
        recovery.run();
        ctrl.lowState->userCmd = UserCommand::R2_A;
        require(recovery.checkChange() == FSMStateName::MJAMP_RECOVERY, "Waiting counted toward standing time");

        // Neither a pre-held start nor latch-generated NONE is a release after entry.
        pose(ctrl, 3.1415926f);
        ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = UserCommand::R2_Y;
        recovery.enter();
        for (int i = 0; i < 5; ++i) {
            recovery.run();
            recovery.checkChange();
            requireDamping(ctrl);
        }
        ctrl.lowState->userCmd = UserCommand::NONE;
        ctrl.lowState->heldUserCmd = UserCommand::R2_X;
        recovery.run();
        recovery.checkChange();
        ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = UserCommand::R2_Y;
        recovery.run();
        requireDamping(ctrl);
        requestStart(recovery, ctrl);
        recovery.run();
        require(ctrl.lowCmd->motorCmd[0].Kp > 0, "Fresh start did not activate lying policy");
        // Waiting must not pre-accumulate the standing switch timer.
        ctrl.lowState->userCmd = UserCommand::R2_A;
        require(recovery.checkChange() == FSMStateName::MJAMP_RECOVERY, "Start bypassed stability timer");

        // Exercise non-upright entry; no fixed-stand prerequisite and no tilt-triggered exit.
        for (float tilt : {1.5707963f, 3.1415926f, -3.1415926f}) {
            pose(ctrl, tilt);
            recovery.enter();
            requestStart(recovery, ctrl);
            recovery.run();
            require(recovery.checkChange() == FSMStateName::MJAMP_RECOVERY, "Lying entry rejected");
            for (const auto &cmd : ctrl.lowCmd->motorCmd)
                require(std::isfinite(cmd.q) && cmd.Kp > 0, "Lying policy action invalid");
        }

        // A rejected held switch must not become an automatic switch later.
        pose(ctrl, 1.0f);
        recovery.enter();
        requestStart(recovery, ctrl);
        recovery.run();
        ctrl.lowState->userCmd = UserCommand::R2_A;
        require(recovery.checkChange() == FSMStateName::MJAMP_RECOVERY, "Tilt guard missing");
        pose(ctrl);
        ctrl.lowState->userCmd = UserCommand::R2_A;
        for (int i = 0; i < 30; ++i) {
            recovery.run();
            require(recovery.checkChange() == FSMStateName::MJAMP_RECOVERY, "Held/rejected switch replayed");
        }
        ctrl.lowState->userCmd = UserCommand::NONE;
        recovery.checkChange();
        ctrl.lowState->userCmd = UserCommand::R2_A;
        require(recovery.checkChange() == FSMStateName::MJAMP, "Stable manual MJAMP switch rejected");
        ctrl.lowState->userCmd = UserCommand::NONE;
        recovery.checkChange();
        ctrl.lowState->userCmd = UserCommand::R2_B;
        require(recovery.checkChange() == FSMStateName::LOCO, "Stable manual LOCO switch rejected");
        ctrl.lowState->imu.gyroscope[0] = 1.0f;
        recovery.run();
        ctrl.lowState->userCmd = UserCommand::NONE;
        recovery.checkChange();
        ctrl.lowState->userCmd = UserCommand::R2_A;
        require(recovery.checkChange() == FSMStateName::MJAMP_RECOVERY, "Angular velocity guard missing");

        // Re-enter all three policies after nonzero actions; first action must be reproducible.
        for (FSMState *state : std::array<FSMState *, 3>{&recovery, &original, &loco}) {
            pose(ctrl);
            state->enter();
            if (state == &recovery) requestStart(recovery, ctrl);
            state->run();
            const auto first = targets(ctrl);
            ctrl.lowState->userValue.ly = 0.8f;
            for (int i = 0; i < 6; ++i) state->run();
            state->exit();
            pose(ctrl);
            state->enter();
            if (state == &recovery) requestStart(recovery, ctrl);
            state->run();
            const auto repeated = targets(ctrl);
            for (int i = 0; i < 29; ++i)
                require(std::abs(first[i] - repeated[i]) < 1e-6f, "Policy re-entry retained stale state");
        }

        pose(ctrl);
        recovery.enter();
        requestStart(recovery, ctrl);
        recovery.run();
        ctrl.lowState->motorState[4].q = std::numeric_limits<float>::quiet_NaN();
        recovery.run();
        requireDamping(ctrl);
        require(recovery.checkChange() == FSMStateName::PASSIVE, "NaN did not request PASSIVE");
        recovery.run();
        requireDamping(ctrl);
        // Even a repeated start cannot clear a fault; only re-entry can reset it.
        ctrl.lowState->motorState[4].q = 0;
        ctrl.lowState->userCmd = ctrl.lowState->heldUserCmd = UserCommand::R2_Y;
        recovery.run();
        requireDamping(ctrl);
        require(recovery.checkChange() == FSMStateName::PASSIVE, "Held start cleared a fault");
        pose(ctrl);
        ctrl.lowState->imu.quaternion[0] = 0;
        recovery.enter();
        requestStart(recovery, ctrl);
        recovery.run();
        requireDamping(ctrl);
        require(recovery.checkChange() == FSMStateName::PASSIVE, "Zero quaternion accepted");
        pose(ctrl);
        recovery.enter();
        requestStart(recovery, ctrl);
        recovery.run();
        ctrl.lowState->imu.gyroscope[1] = std::numeric_limits<float>::infinity();
        recovery.run();
        requireDamping(ctrl);
        pose(ctrl);
        recovery.enter();
        requestStart(recovery, ctrl);
        recovery.run();
        ctrl.lowState->userValue.ly = std::numeric_limits<float>::quiet_NaN();
        recovery.run();
        requireDamping(ctrl);
        pose(ctrl);
        recovery.enter();
        ctrl.lowState->userCmd = UserCommand::L2_B;
        recovery.run();
        requireDamping(ctrl);
        require(recovery.checkChange() == FSMStateName::PASSIVE, "Manual damping exit missing");
        pose(ctrl);
        recovery.enter();
        requestStart(recovery, ctrl);
        recovery.run();
        ctrl.lowState->userCmd = UserCommand::L2_B;
        recovery.run();
        requireDamping(ctrl);
        require(recovery.checkChange() == FSMStateName::PASSIVE, "Running damping exit missing");
        pose(ctrl);
        recovery.enter();
        ctrl.lowState->userCmd = UserCommand::SELECT;
        recovery.run();
        requireDamping(ctrl);
        bool selected = false;
        try { recovery.checkChange(); } catch (const std::runtime_error &) { selected = true; }
        require(selected, "SELECT exit missing");
        pose(ctrl);
        ctrl.dt = 0.01;
        recovery.enter();
        requestStart(recovery, ctrl);
        recovery.run();
        requireDamping(ctrl);
        ctrl.dt = 0.02;

        // Raw sensor/command fixtures for an independent Python training-config oracle.
        json trace = json::array();
        auto sample = [&](bool reset, std::array<float, 3> expectedCommand) {
            auto &s = *ctrl.lowState;
            json row{{"reset", reset}, {"quat", s.imu.quaternion}, {"gyro", s.imu.gyroscope},
                     {"command", expectedCommand}};
            for (const auto &motor : s.motorState) {
                row["q"].push_back(motor.q);
                row["dq"].push_back(motor.dq);
            }
            if (reset) {
                // Enter waiting in a different pose, then reposition before activation.
                // The independent oracle below must see only the activation-time history.
                const auto activationState = s;
                pose(ctrl, -1.0f);
                recovery.enter();
                recovery.run();
                requireDamping(ctrl);
                s = activationState;
                requestStart(recovery, ctrl);
            }
            recovery.run();
            require(recovery.checkChange() == FSMStateName::MJAMP_RECOVERY, "Trace policy terminated");
            for (const auto &motor : ctrl.lowCmd->motorCmd) {
                row["target"].push_back(motor.q);
                row["kp"].push_back(motor.Kp);
                row["kd"].push_back(motor.Kd);
            }
            trace.push_back(row);
        };
        pose(ctrl, 0.4f);
        ctrl.lowState->userValue.ly = 0.8f;
        sample(true, {0, 0, 0});
        sample(false, {0, 0, 0}); // Held stick is not armed.
        ctrl.lowState->userValue.setZero();
        sample(false, {0, 0, 0}); // Centering arms commands.
        ctrl.lowState->userValue.ly = 0.5f;
        ctrl.lowState->userValue.lx = -0.4f;
        ctrl.lowState->userValue.rx = 0.3f;
        sample(false, {0.5f, -0.4f, 0.471f});
        ctrl.lowState->userCmd = UserCommand::R2_UP;
        recovery.checkChange();
        for (int step = 0; step < 12; ++step) {
            for (int j = 0; j < 29; ++j) {
                ctrl.lowState->motorState[j].q += 0.003f * std::sin(float(step + j));
                ctrl.lowState->motorState[j].dq = 0.06f * std::cos(float(step + j));
            }
            sample(false, {1.25f, -0.4f, 0.471f});
        }
        pose(ctrl, 3.1415926f);
        sample(true, {0, 0, 0});
        sample(false, {0, 0, 0});
        if (argc > 1) {
            std::ofstream output(argv[1]);
            output << trace.dump(2) << '\n';
            require(bool(output), "Cannot write oracle trace");
        }
        std::cout << "PASS: recovery entry, switch guards, re-entry, faults, buttons; ORT "
                  << Ort::GetVersionString() << std::endl;
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << std::endl;
        return 1;
    }
}
