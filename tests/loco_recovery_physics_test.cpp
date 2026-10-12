#include "FSM/State_LocoRecovery.h"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Fully local simulation of the production policies and bridge torque formula.
// No window, DDS, hardware connection, or control of the user's running scene.
void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}

struct Jump { double target = 0, torque = 0; };

Jump commandJump(const LowlevelCmd &before, const LowlevelCmd &after,
                 const LowlevelState &feedback)
{
    Jump result;
    for (int i = 0; i < 29; ++i) {
        const auto &a = before.motorCmd[i], &b = after.motorCmd[i];
        const auto &joint = feedback.motorState[i];
        auto pd = [&](const MotorCmd &cmd) {
            return double(cmd.tau) + cmd.Kp * (double(cmd.q) - joint.q) +
                   cmd.Kd * (double(cmd.dq) - joint.dq);
        };
        result.target = std::max(result.target, std::abs(double(b.q) - a.q));
        result.torque = std::max(result.torque, std::abs(pd(b) - pd(a)));
    }
    return result;
}

void sameCommand(const LowlevelCmd &actual, const LowlevelCmd &raw)
{
    for (int i = 0; i < 29; ++i) {
        const auto &a = actual.motorCmd[i], &b = raw.motorCmd[i];
        require(a.mode == b.mode && std::abs(a.q - b.q) < 1e-5 &&
                std::abs(a.dq - b.dq) < 1e-5 && std::abs(a.tau - b.tau) < 1e-5 &&
                std::abs(a.Kp - b.Kp) < 1e-4 && std::abs(a.Kd - b.Kd) < 1e-5,
                "Urgent takeover delayed or mixed the raw Recovery command");
    }
}

struct PushCase {
    const char *name;
    double forceX, forceY;
    bool smooth;
};

void runCase(const PushCase &push)
{
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(RECOVERY_TEST_SCENE, nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) throw std::runtime_error(error);
    require(model->nu == 29 && model->nq == 36, "Expected the floating-base G1 29-DOF scene");
    model->opt.timestep = 0.002;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
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
    CtrlComponents ctrl(nullptr), reference(nullptr);
    ctrl.dt = reference.dt = 0.02;
    State_Loco loco(&ctrl);
    State_MJAMP_RECOVERY recovery(&ctrl), shadowRecovery(&reference);
    State_LocoRecovery state(&ctrl, loco, recovery);
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
    feedback(); state.enter();
    bool recoverySeen = false, blendSeen = false, returned = false, walkingStarted = false;
    bool walkingLocoSustained = true;
    double walkX = 0, walkY = 0, minimumHeight = 1;
    double recoveryTime = 0, returnTime = 0, tiltAtTakeover = 0, rateAtTakeover = 0;
    Jump firstSent, firstRaw, peakSent, peakRaw;
    LowlevelCmd previousRaw;
    int shadowFrames = 0;
    bool urgentCommandsEqual = true;
    for (int tick = 0; tick < 1250; ++tick) {
        feedback();
        const double time = data->time;
        if (time >= 15.0) {
            if (!walkingStarted) {
                walkingStarted = true;
                walkX = data->xpos[3 * pelvis];
                walkY = data->xpos[3 * pelvis + 1];
            }
            ctrl.lowState->userValue.ly = 0.3;
        }
        const LowlevelCmd before = *ctrl.lowCmd;
        state.run();
        require(state.checkChange() == FSMStateName::LOCO_RECOVERY,
                "Physics recovery withdrew joint support or exited the composite");
        if (time >= 15.0 && state.phase() != State_LocoRecovery::Phase::LOCO)
            walkingLocoSustained = false;
        if (time > 3 && time < 15)
            minimumHeight = std::min(minimumHeight, data->xpos[3 * pelvis + 2]);
        const bool takeover = state.phase() == State_LocoRecovery::Phase::RECOVERY && !recoverySeen;
        if (takeover) {
            recoverySeen = true;
            recoveryTime = time;
            std::vector<float> quat(ctrl.lowState->imu.quaternion, ctrl.lowState->imu.quaternion + 4);
            const auto gravity = QuatRotateInverse(quat, {0, 0, -1});
            tiltAtTakeover = std::acos(std::clamp(-double(gravity[2]), -1.0, 1.0));
            rateAtTakeover = (ctrl.lowState->imu.gyroscope[1] * gravity[0] -
                ctrl.lowState->imu.gyroscope[0] * gravity[1]) / std::hypot(gravity[0], gravity[1]);
            *reference.lowState = *ctrl.lowState;
            shadowRecovery.enterAutomatic();
            previousRaw = before;
        } else if (recoverySeen && shadowFrames < 10) {
            *reference.lowState = *ctrl.lowState;
            shadowRecovery.run();
        }
        if (recoverySeen && shadowFrames < 10) {
            require(!shadowRecovery.failed(), "Shadow Recovery failed on the physical feedback sequence");
            require(state.phase() == State_LocoRecovery::Phase::RECOVERY,
                    "Recovery exited during the first 200 ms measurement window");
            const auto sentJump = commandJump(before, *ctrl.lowCmd, *ctrl.lowState);
            const auto rawJump = commandJump(previousRaw, *reference.lowCmd, *ctrl.lowState);
            if (shadowFrames == 0) { firstSent = sentJump; firstRaw = rawJump; }
            peakSent.target = std::max(peakSent.target, sentJump.target);
            peakSent.torque = std::max(peakSent.torque, sentJump.torque);
            peakRaw.target = std::max(peakRaw.target, rawJump.target);
            peakRaw.torque = std::max(peakRaw.torque, rawJump.torque);
            if (!push.smooth) {
                try { sameCommand(*ctrl.lowCmd, *reference.lowCmd); }
                catch (const std::runtime_error &) { urgentCommandsEqual = false; }
            }
            previousRaw = *reference.lowCmd;
            ++shadowFrames;
        }
        if (recoverySeen && state.phase() == State_LocoRecovery::Phase::RETURNING) blendSeen = true;
        if (blendSeen && state.phase() == State_LocoRecovery::Phase::LOCO && !returned) {
            returned = true;
            returnTime = time;
        }
        for (int substep = 0; substep < 10; ++substep) {
            for (int i = 0; i < 29; ++i) {
                const auto &cmd = ctrl.lowCmd->motorCmd[i];
                data->ctrl[i] = cmd.tau + cmd.Kp * (cmd.q - data->sensordata[i]) +
                               cmd.Kd * (cmd.dq - data->sensordata[29 + i]);
            }
            const bool pushing = data->time >= 3 && data->time < 3.5;
            data->xfrc_applied[6 * torso] = pushing ? push.forceX : 0;
            data->xfrc_applied[6 * torso + 1] = pushing ? push.forceY : 0;
            mj_step(model.get(), data.get());
        }
    }
    const double distance = std::hypot(data->xpos[3 * pelvis] - walkX,
                                      data->xpos[3 * pelvis + 1] - walkY);
    const double finalHeight = data->xpos[3 * pelvis + 2];
    const bool finalLoco = state.phase() == State_LocoRecovery::Phase::LOCO;
    std::cout << "RESULT " << push.name << ": recovery=" << recoveryTime
              << "s, return=" << returnTime << "s, tilt=" << tiltAtTakeover
              << "rad, outward_rate=" << rateAtTakeover << "rad/s, first_q_jump="
              << firstSent.target << " (shadow_raw=" << firstRaw.target
              << "), first_estimated_PD_jump=" << firstSent.torque
              << " (shadow_raw=" << firstRaw.torque << "), first_200ms_peak_q_jump="
              << peakSent.target << " (shadow_raw=" << peakRaw.target
              << "), first_200ms_peak_estimated_PD_jump=" << peakSent.torque
              << " (shadow_raw=" << peakRaw.torque << "), minimum_pelvis_height="
              << minimumHeight << "m, walk_distance=" << distance
              << "m, final_pelvis_height=" << finalHeight << "m\n";
    state.exit();
    require(recoverySeen && shadowFrames == 10, "Physical push did not exercise the Recovery handoff");
    if (push.smooth) {
        require(tiltAtTakeover < 0.73303829 && rateAtTakeover < 2.5,
                "Mild push did not exercise the smoothing branch");
        require(firstSent.target < 0.6 * firstRaw.target && firstSent.torque < 0.85 * firstRaw.torque,
                "Mild takeover did not reduce the first target/estimated PD command jump");
        require(peakSent.torque < 0.8 * peakRaw.torque,
                "Mild takeover only deferred the estimated PD jump within the first 200 ms");
    } else {
        require(tiltAtTakeover >= 0.73303829 || rateAtTakeover >= 2.5,
                "Urgent push did not exercise immediate Recovery authority");
        require(urgentCommandsEqual, "Urgent takeover delayed or mixed the raw Recovery command");
        require(minimumHeight < 0.5, "Urgent physical fall/get-up sequence was not exercised");
    }
    require(returned && returnTime < 15,
            "Recovery did not return to Loco before the walking command");
    require(finalLoco && walkingLocoSustained && distance > 0.5 && finalHeight > 0.65,
            "Recovered Loco did not walk upright in response to joystick input");
}

int main()
{
    const std::array<PushCase, 4> cases{{
        {"front_200N_smooth", 200, 0, true},
        {"back_200N_smooth", -200, 0, true},
        {"front_250N_urgent", 250, 0, false},
        {"side_250N_urgent", 0, 250, false}}};
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
        std::cout << "PASS: four physical handoff/recovery/walking cases (headless MuJoCo, production ONNX, no DDS). "
                     "Shadow commands share the actual feedback trajectory; PD estimates are not measured/clipped motor torques.\n";
    return failures == 0 ? 0 : 1;
}
