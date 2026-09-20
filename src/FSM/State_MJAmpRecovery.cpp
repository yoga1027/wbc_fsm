#include "FSM/State_MJAmpRecovery.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace {
using json = nlohmann::json;

json readConfig(const std::string &path)
{
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open config: " + path);
    return json::parse(file);
}

float finiteNumber(const json &cfg, const char *key)
{
    const float value = cfg.at(key).get<float>();
    if (!std::isfinite(value)) throw std::runtime_error(std::string("Invalid config: ") + key);
    return value;
}

std::optional<float> clipLimit(const json &cfg, const char *key)
{
    if (cfg.at(key).is_null()) return std::nullopt;
    const float limit = finiteNumber(cfg, key);
    if (limit <= 0) throw std::runtime_error(std::string("Clip must be positive: ") + key);
    return limit;
}

float clipped(float value, const std::optional<float> &limit)
{
    return limit ? std::clamp(value, -*limit, *limit) : value;
}
} // namespace

State_MJAMP_RECOVERY::State_MJAMP_RECOVERY(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::MJAMP_RECOVERY, "mjamp_recovery")
{
    _loadConfig();
    for (int i = 0; i < kNumDof; ++i)
        dof_action_scale[i] = 0.25 * limit_dof_tau[i] / dof_Kps[i];
    _loadPolicy();
}

void State_MJAMP_RECOVERY::_loadConfig()
{
    const std::string root = std::string(PROJECT_ROOT_DIR) + "/";
    const auto cfg = readConfig(root + "config/mjamp_recovery.json");
    _modelPath = root + cfg.at("model_path").get<std::string>();
    auto range = [&](const char *low, const char *high) {
        std::array<float, 2> result{finiteNumber(cfg, low), finiteNumber(cfg, high)};
        if (result[0] > 0 || result[1] < 0 || result[0] > result[1])
            throw std::runtime_error("Command range must contain zero");
        return result;
    };
    _vxLimit = range("vx_limit_min", "vx_limit_max");
    _vxSlowLimit = range("vx_limit_min_slow", "vx_limit_max_slow");
    _vyLimit = range("vy_limit_min", "vy_limit_max");
    _yawLimit = range("wyaw_limit_min", "wyaw_limit_max");
    _commandSmoothing = finiteNumber(cfg, "cmd_smoothes");
    _clipObservations = clipLimit(cfg, "clip_observations");
    _clipActions = clipLimit(cfg, "clip_actions");
    _enableOrientationExit = cfg.at("enable_orientation_exit").get<bool>();
    _orientationExitAngle = finiteNumber(cfg, "orientation_exit_angle_rad");
    _switchMaxTilt = finiteNumber(cfg, "switch_max_tilt_rad");
    _switchMaxAngularVelocity = finiteNumber(cfg, "switch_max_angular_velocity");
    _switchHoldSeconds = finiteNumber(cfg, "switch_hold_seconds");
    _passiveKd = finiteNumber(readConfig(root + "config/passive.json"), "passive_kds");
    if (_commandSmoothing < 0 || _commandSmoothing >= 1 || _passiveKd < 0 ||
        _orientationExitAngle <= 0 || _orientationExitAngle > 3.141593f ||
        _switchMaxTilt <= 0 || _switchMaxTilt >= 1.570797f ||
        _switchMaxAngularVelocity <= 0 || _switchHoldSeconds <= 0)
        throw std::runtime_error("Invalid MJAMP_RECOVERY configuration");
}

void State_MJAMP_RECOVERY::_loadPolicy()
{
    _sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    _sessionOptions.SetIntraOpNumThreads(1);
    _session = std::make_unique<Ort::Session>(_env, _modelPath.c_str(), _sessionOptions);
    if (_session->GetInputCount() != 1 || _session->GetOutputCount() != 1)
        throw std::runtime_error("MJAMP_RECOVERY expects one input and one output");
    Ort::AllocatorWithDefaultOptions allocator;
    const auto inputName = _session->GetInputNameAllocated(0, allocator);
    const auto outputName = _session->GetOutputNameAllocated(0, allocator);
    const auto inputType = _session->GetInputTypeInfo(0);
    const auto outputType = _session->GetOutputTypeInfo(0);
    const auto input = inputType.GetTensorTypeAndShapeInfo();
    const auto output = outputType.GetTensorTypeAndShapeInfo();
    auto validShape = [](const std::vector<int64_t> &shape, int width) {
        return shape.size() == 2 && (shape[0] == 1 || shape[0] == -1) && shape[1] == width;
    };
    if (std::string(inputName.get()) != "obs" || std::string(outputName.get()) != "actions" ||
        input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        !validShape(input.GetShape(), kObsSize) || !validShape(output.GetShape(), kNumDof))
        throw std::runtime_error("MJAMP_RECOVERY requires float32 obs[1,384] -> actions[1,29]");
    std::cout << "[MJAMP_RECOVERY] Model: " << _modelPath << std::endl;
}

void State_MJAMP_RECOVERY::_updateCommand()
{
    _userValue = _lowState->userValue;
    const std::array<float, 3> sticks{_userValue.ly, _userValue.lx, _userValue.rx};
    for (float value : sticks)
        if (!std::isfinite(value)) throw std::runtime_error("Non-finite joystick input");
    if (!_commandArmed) {
        _commandArmed = std::all_of(sticks.begin(), sticks.end(), [](float v) {
            return std::abs(v) <= kDeadZone;
        });
        _command.fill(0.0f);
        _previousCommand.fill(0.0f);
        return;
    }
    const std::array<std::array<float, 2>, 3> limits{
        _highSpeed ? _vxLimit : _vxSlowLimit, _vyLimit, _yawLimit};
    for (int i = 0; i < 3; ++i) {
        const float stick = std::clamp(sticks[i], -1.0f, 1.0f);
        const float requested = std::abs(stick) <= kDeadZone ? 0.0f :
            stick * (stick < 0 ? -limits[i][0] : limits[i][1]);
        _command[i] = _previousCommand[i] * _commandSmoothing +
            requested * (1.0f - _commandSmoothing);
    }
    _previousCommand = _command;
}

std::array<float, State_MJAMP_RECOVERY::kFrameSize> State_MJAMP_RECOVERY::_readFrame()
{
    std::vector<float> quat(_lowState->imu.quaternion, _lowState->imu.quaternion + 4);
    float normSquared = 0.0f;
    for (float v : quat) normSquared += v * v;
    if (!std::isfinite(normSquared) || normSquared < 0.25f || normSquared > 2.25f)
        throw std::runtime_error("Invalid IMU quaternion");
    const float norm = std::sqrt(normSquared);
    for (float &v : quat) v /= norm;
    const auto gravity = QuatRotateInverse(quat, {0.0f, 0.0f, -1.0f});
    std::array<float, kFrameSize> frame{};
    float angularSpeedSquared = 0.0f;
    for (int i = 0; i < 3; ++i) {
        frame[i] = _lowState->imu.gyroscope[i];
        angularSpeedSquared += frame[i] * frame[i];
        frame[3 + i] = gravity[i];
        frame[6 + i] = _command[i];
    }
    for (int i = 0; i < kNumDof; ++i) {
        const int motor = dof_mapping_mj[i];
        frame[9 + i] = _lowState->motorState[motor].q - _default_dof_pos[motor];
        frame[38 + i] = _lowState->motorState[motor].dq;
        frame[67 + i] = _action[i];
    }
    for (float &value : frame) {
        if (!std::isfinite(value)) throw std::runtime_error("Non-finite recovery observation");
        value = clipped(value, _clipObservations);
    }
    const float tilt = std::acos(std::clamp(-gravity[2], -1.0f, 1.0f));
    if (_enableOrientationExit && tilt > _orientationExitAngle)
        throw std::runtime_error("Recovery orientation exit threshold exceeded");
    // Consecutive 50 Hz samples, not just one momentarily upright observation.
    if (tilt < _switchMaxTilt && std::sqrt(angularSpeedSquared) < _switchMaxAngularVelocity)
        _stableSeconds = std::min(_stableSeconds + _ctrlComp->dt, double(_switchHoldSeconds));
    else
        _stableSeconds = 0.0;
    return frame;
}

void State_MJAMP_RECOVERY::_inferAndWriteCommand()
{
    const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
    const std::array<int64_t, 2> shape{1, kObsSize};
    auto input = Ort::Value::CreateTensor<float>(memory, _observation.data(),
        _observation.size(), shape.data(), shape.size());
    const char *inputNames[] = {"obs"};
    const char *outputNames[] = {"actions"};
    auto outputs = _session->Run(Ort::RunOptions{nullptr}, inputNames, &input, 1, outputNames, 1);
    if (!outputs[0].IsTensor() ||
        outputs[0].GetTensorTypeAndShapeInfo().GetShape() != std::vector<int64_t>({1, kNumDof}))
        throw std::runtime_error("Unexpected recovery output shape");
    const float *result = outputs[0].GetTensorData<float>();
    std::array<float, kNumDof> nextAction{}, target{};
    for (int i = 0; i < kNumDof; ++i) {
        if (!std::isfinite(result[i])) throw std::runtime_error("Non-finite recovery action");
        const int motor = dof_mapping_mj[i];
        nextAction[i] = clipped(result[i], _clipActions);
        target[motor] = _default_dof_pos[motor] + nextAction[i] * dof_action_scale[motor];
        if (!std::isfinite(target[motor])) throw std::runtime_error("Non-finite recovery target");
    }
    // Validate all joints before publishing any part of this action.
    _action = nextAction;
    for (int i = 0; i < kNumDof; ++i) {
        auto &cmd = _lowCmd->motorCmd[i];
        cmd.mode = 10;
        cmd.q = target[i];
        cmd.dq = 0;
        cmd.tau = 0;
        cmd.Kp = dof_Kps[i];
        cmd.Kd = dof_Kds[i];
    }
}

void State_MJAMP_RECOVERY::_writeDamping()
{
    for (int i = 0; i < kNumDof; ++i) {
        auto &cmd = _lowCmd->motorCmd[i];
        cmd.mode = 10;
        cmd.q = 0;
        cmd.dq = 0;
        cmd.tau = 0;
        cmd.Kp = 0;
        cmd.Kd = _passiveKd;
    }
}

void State_MJAMP_RECOVERY::_fail(const std::string &reason)
{
    if (!_terminate) std::cerr << "[MJAMP_RECOVERY] " << reason << "; switching to PASSIVE" << std::endl;
    _terminate = true;
    _stableSeconds = 0;
    _writeDamping();
}

void State_MJAMP_RECOVERY::_resetPolicyBuffers()
{
    _action.fill(0.0f);
    _observation.fill(0.0f);
    _command.fill(0.0f);
    _previousCommand.fill(0.0f);
    _highSpeed = false;
    _commandArmed = false;
    _stableSeconds = 0;
}

void State_MJAMP_RECOVERY::enter()
{
    _resetPolicyBuffers();
    _terminate = false;
    _phase = Phase::WAITING;
    _startReleased = false;
    _lastUserCommand = _lowState->userCmd;
    _writeDamping();
    std::cout << "[MJAMP_RECOVERY] WAITING: damping only. Release buttons, then press R2+Y to start."
              << std::endl;
}

void State_MJAMP_RECOVERY::_startPolicy()
{
    // The robot may have been repositioned while waiting. Initialize at activation,
    // never reuse entry-time history or actions from a previous run.
    _resetPolicyBuffers();
    if (!std::isfinite(_ctrlComp->dt) || std::abs(_ctrlComp->dt - 0.02) > 1e-6)
        throw std::runtime_error("Recovery policy requires dt = 0.02 s");
    const auto frame = _readFrame();
    for (int i = 0; i < kHistoryLength; ++i)
        std::copy(frame.begin(), frame.end(), _observation.begin() + i * kFrameSize);
    _stableSeconds = 0; // Waiting/backfill does not count toward switch readiness.
    _inferAndWriteCommand(); // Commit RUNNING only after the first action is valid.
    _phase = Phase::RUNNING;
    _startReleased = false;
    std::cout << "[MJAMP_RECOVERY] RUNNING: policy started from current pose; center sticks to enable commands."
              << std::endl;
}

void State_MJAMP_RECOVERY::run()
{
    if (_terminate || _lowState->userCmd == UserCommand::L2_B ||
        _lowState->userCmd == UserCommand::SELECT) {
        _writeDamping();
        return;
    }
    try {
        if (_phase == Phase::WAITING) {
            _writeDamping();
            // userCmd alone is insufficient: held R2+X is filtered to NONE by IOSDK.
            if (_lowState->heldUserCmd == UserCommand::NONE &&
                _lowState->userCmd == UserCommand::NONE)
                _startReleased = true;
            if (_startReleased && _lowState->userCmd == UserCommand::R2_Y)
                _startPolicy();
            return;
        }
        _updateCommand();
        const auto frame = _readFrame();
        std::move(_observation.begin() + kFrameSize, _observation.end(), _observation.begin());
        std::copy(frame.begin(), frame.end(), _observation.end() - kFrameSize);
        _inferAndWriteCommand();
    } catch (const std::exception &error) {
        _fail(error.what());
    }
}

void State_MJAMP_RECOVERY::exit()
{
    _resetPolicyBuffers();
    _phase = Phase::WAITING;
    _startReleased = false;
    std::cout << "[MJAMP_RECOVERY] Exiting recovery policy" << std::endl;
}

FSMStateName State_MJAMP_RECOVERY::checkChange()
{
    const auto command = _lowState->userCmd;
    const bool fresh = command != _lastUserCommand;
    _lastUserCommand = command;
    if (command == UserCommand::L2_B || _terminate) return FSMStateName::PASSIVE;
    if (command == UserCommand::SELECT) throw std::runtime_error("exit..");
    // Do not bypass the explicit start via MJAMP/LOCO or count waiting as standing.
    if (_phase == Phase::WAITING) return FSMStateName::MJAMP_RECOVERY;
    if (command == UserCommand::R2_UP) _highSpeed = true;
    if (command == UserCommand::R2_DOWN) _highSpeed = false;
    if (fresh && (command == UserCommand::R2_A || command == UserCommand::R2_B)) {
        if (_stableSeconds >= _switchHoldSeconds)
            return command == UserCommand::R2_A ? FSMStateName::MJAMP : FSMStateName::LOCO;
        std::cout << "[MJAMP_RECOVERY] Switch blocked: wait for low tilt/angular velocity, "
                     "then release and press the combination again." << std::endl;
    }
    return FSMStateName::MJAMP_RECOVERY;
}
