#include "FSM/State_LocoRecovery.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <stdexcept>

namespace {
nlohmann::json config(const char *name)
{
    std::ifstream input(std::string(PROJECT_ROOT_DIR) + "/config/" + name);
    if (!input) throw std::runtime_error(std::string("Cannot open config: ") + name);
    return nlohmann::json::parse(input);
}
double positive(const nlohmann::json &cfg, const char *key)
{
    const double value = cfg.at(key).get<double>();
    if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error(std::string("Invalid positive parameter: ") + key);
    return value;
}

double smoothstep(double progress)
{
    return progress * progress * (3.0 - 2.0 * progress);
}

void blendCommand(const LowlevelCmd &from, LowlevelCmd &to, double alpha)
{
    for (int i = 0; i < 29; ++i) {
        auto &cmd = to.motorCmd[i];
        const auto &start = from.motorCmd[i];
        cmd.q = (1 - alpha) * start.q + alpha * cmd.q;
        cmd.dq = (1 - alpha) * start.dq + alpha * cmd.dq;
        cmd.tau = (1 - alpha) * start.tau + alpha * cmd.tau;
        cmd.Kp = (1 - alpha) * start.Kp + alpha * cmd.Kp;
        cmd.Kd = (1 - alpha) * start.Kd + alpha * cmd.Kd;
    }
}

struct CommandJump { double target = 0, torque = 0; };
CommandJump commandJump(const LowlevelCmd &before, const LowlevelCmd &after,
                        const LowlevelState &state)
{
    CommandJump jump;
    for (int i = 0; i < 29; ++i) {
        const auto &a = before.motorCmd[i], &b = after.motorCmd[i];
        const auto &joint = state.motorState[i];
        auto pdTorque = [&](const MotorCmd &cmd) {
            return double(cmd.tau) + cmd.Kp * (double(cmd.q) - joint.q) +
                   cmd.Kd * (double(cmd.dq) - joint.dq);
        };
        jump.target = std::max(jump.target, std::abs(double(b.q) - a.q));
        jump.torque = std::max(jump.torque, std::abs(pdTorque(b) - pdTorque(a)));
    }
    return jump;
}
}

State_LocoRecovery::State_LocoRecovery(CtrlComponents *ctrl, State_Loco &loco,
                                     State_MJAMP_RECOVERY &recovery)
    : FSMState(ctrl, FSMStateName::LOCO_RECOVERY, "RL Loco+MJAmpRecovery"),
      _loco(loco), _recovery(recovery)
{
    const auto cfg = config("loco_recovery.json");
    _triggerTilt = positive(cfg, "trigger_tilt_rad");
    _hardTilt = positive(cfg, "hard_tilt_rad");
    _triggerRate = positive(cfg, "trigger_tilt_rate_rad_s");
    _triggerHold = positive(cfg, "trigger_hold_seconds");
    _stableTilt = positive(cfg, "stable_tilt_rad");
    _stableAngularSpeed = positive(cfg, "stable_angular_velocity");
    _stableJointSpeed = positive(cfg, "stable_joint_velocity");
    _stableHold = positive(cfg, "stable_hold_seconds");
    _minimumRecovery = positive(cfg, "minimum_recovery_seconds");
    // Older local configs used a hard timeout. Treat that value as a warning
    // interval too: elapsed time alone must never withdraw standing support.
    _recoveryWarningSeconds = positive(cfg, cfg.contains("recovery_warning_seconds") ?
        "recovery_warning_seconds" : "recovery_timeout_seconds");
    _returnDuration = positive(cfg, "return_blend_seconds");
    _recoveryBlendMaxDuration = cfg.value("recovery_blend_seconds", 0.08);
    _recoveryBlendFastTiltRate = cfg.value("recovery_blend_fast_tilt_rate", 2.5);
    _recoveryBlendTimeMargin = cfg.value("recovery_blend_time_margin", 0.5);
    if (!std::isfinite(_recoveryBlendMaxDuration) || _recoveryBlendMaxDuration < 0 ||
        _recoveryBlendMaxDuration > 0.15 || !std::isfinite(_recoveryBlendFastTiltRate) ||
        _recoveryBlendFastTiltRate <= _triggerRate || !std::isfinite(_recoveryBlendTimeMargin) ||
        _recoveryBlendTimeMargin <= 0 || _recoveryBlendTimeMargin > 1)
        throw std::runtime_error("Invalid recovery handoff duration/risk configuration");
    _commandRamp = positive(cfg, "command_ramp_seconds");
    _stickDeadZone = positive(cfg, "stick_dead_zone");
    _passiveKd = positive(config("passive.json"), "passive_kds");
    _standingMin = cfg.at("standing_joint_min").get<std::array<double, 12>>();
    _standingMax = cfg.at("standing_joint_max").get<std::array<double, 12>>();
    for (int i = 0; i < 12; ++i)
        if (!std::isfinite(_standingMin[i]) || !std::isfinite(_standingMax[i]) ||
            _standingMin[i] >= _standingMax[i])
            throw std::runtime_error("Invalid standing joint range");
    // The composite must catch tilt before the ordinary Loco termination guard.
    const double locoLimit = positive(config("loco.json"), "safe_projgravity_threshold");
    if (!(_stableTilt < _triggerTilt && _triggerTilt < _hardTilt &&
          _hardTilt < 1.57079632679 && 1.0 - std::cos(_hardTilt) < locoLimit) ||
        _stickDeadZone >= 1 || _recoveryWarningSeconds <= _minimumRecovery + _stableHold)
        throw std::runtime_error("Invalid Loco recovery hysteresis/timing configuration");
}

State_LocoRecovery::Sample State_LocoRecovery::_readSample() const
{
    if (!std::isfinite(_ctrlComp->dt) || std::abs(_ctrlComp->dt - 0.02) > 1e-6)
        throw std::runtime_error("Loco recovery requires dt = 0.02 s");
    std::vector<float> quat(_lowState->imu.quaternion, _lowState->imu.quaternion + 4);
    double norm2 = 0;
    for (float v : quat) norm2 += double(v) * v;
    if (!std::isfinite(norm2) || norm2 < 0.25 || norm2 > 2.25)
        throw std::runtime_error("Invalid IMU quaternion");
    for (float &v : quat) v /= std::sqrt(norm2);
    const auto gravity = QuatRotateInverse(quat, {0, 0, -1});
    double speed2 = 0;
    for (float v : _lowState->imu.gyroscope) {
        if (!std::isfinite(v)) throw std::runtime_error("Invalid IMU angular velocity");
        speed2 += double(v) * v;
    }
    for (const auto &joint : _lowState->motorState)
        if (!std::isfinite(joint.q) || !std::isfinite(joint.dq))
            throw std::runtime_error("Invalid joint feedback");
    const auto &stick = _lowState->userValue;
    if (!std::isfinite(stick.lx) || !std::isfinite(stick.ly) || !std::isfinite(stick.rx))
        throw std::runtime_error("Invalid joystick input");
    const double tilt = std::acos(std::clamp(-double(gravity[2]), -1.0, 1.0));
    // g_dot = -omega x g. This is the outward tilt rate, excluding yaw;
    // inward correction, even with a large gyro reading, is not an early trigger.
    const double sinTilt = std::hypot(gravity[0], gravity[1]);
    const double rate = sinTilt > 1e-5 ?
        (_lowState->imu.gyroscope[1] * gravity[0] -
         _lowState->imu.gyroscope[0] * gravity[1]) / sinTilt : 0.0;
    return {tilt, rate, std::sqrt(speed2)};
}

bool State_LocoRecovery::_falling(const Sample &sample)
{
    if (sample.tilt >= _hardTilt) return true;
    if (sample.tilt >= _triggerTilt && sample.tiltRate >= _triggerRate)
        _riskSeconds += _ctrlComp->dt;
    else
        _riskSeconds = 0;
    return _riskSeconds + 1e-9 >= _triggerHold;
}

bool State_LocoRecovery::_standing(const Sample &sample) const
{
    if (sample.tilt > _stableTilt || sample.angularSpeed > _stableAngularSpeed) return false;
    for (int i = 0; i < 12; ++i) {
        const auto &joint = _lowState->motorState[i];
        if (joint.q < _standingMin[i] || joint.q > _standingMax[i] ||
            std::abs(joint.dq) > _stableJointSpeed) return false;
    }
    return true;
}

void State_LocoRecovery::_reportRecovery(const Sample &sample)
{
    if (_recoverySeconds + 1e-9 < _nextRecoveryReport) return;
    _nextRecoveryReport = _recoverySeconds + 2.0;
    std::cout << "[LocoRecovery] RECOVERY t=" << _recoverySeconds
              << "s, tilt=" << sample.tilt << "/" << _stableTilt
              << " rad, angular_speed=" << sample.angularSpeed << "/" << _stableAngularSpeed
              << ", stable_hold=" << _stableSeconds << "/" << _stableHold << "s";
    if (sample.tilt > _stableTilt) std::cout << ", waiting for upright torso";
    else if (sample.angularSpeed > _stableAngularSpeed) std::cout << ", waiting for low angular speed";
    else {
        for (int i = 0; i < 12; ++i) {
            const auto &joint = _lowState->motorState[i];
            if (joint.q < _standingMin[i] || joint.q > _standingMax[i]) {
                std::cout << ", joint[" << i << "].q=" << joint.q
                          << " outside [" << _standingMin[i] << "," << _standingMax[i] << "]";
                break;
            }
            if (std::abs(joint.dq) > _stableJointSpeed) {
                std::cout << ", joint[" << i << "].dq=" << joint.dq
                          << " exceeds " << _stableJointSpeed;
                break;
            }
        }
    }
    std::cout << "; velocity commands held at zero until return" << std::endl;
}

void State_LocoRecovery::_damping()
{
    for (auto &cmd : _lowCmd->motorCmd) {
        cmd.mode = 10;
        cmd.q = cmd.dq = cmd.tau = cmd.Kp = 0;
        cmd.Kd = _passiveKd;
    }
}

void State_LocoRecovery::_fail(const std::string &reason)
{
    if (!_failed) std::cerr << "[LocoRecovery] " << reason << "; PASSIVE" << std::endl;
    _failed = true;
    _damping();
}

void State_LocoRecovery::_startRecovery(const Sample &sample)
{
    // FSM sendRecv() has just sent this buffer. Preserve that actual command,
    // including an unfinished return blend, before Recovery overwrites it.
    _recoveryBlendFrom = *_lowCmd;
    _recoveryBlendSeconds = 0;
    const bool supported = _policyActive &&
        std::all_of(std::begin(_lowCmd->motorCmd), std::end(_lowCmd->motorCmd),
                    [](const MotorCmd &cmd) { return cmd.Kp > 0; });
    _recoveryBlendDuration = supported ? _recoveryBlendBudget(sample) : 0;
    if (_policyActive) _loco.exit();
    _phase = Phase::RECOVERY;
    _policyActive = true;
    _riskSeconds = _stableSeconds = _recoverySeconds = 0;
    _nextRecoveryReport = 2.0;
    _recoveryWarned = false;
    _commandsArmed = false;
    _commandSeconds = 0;
    _recovery.enterAutomatic(); // Computes the first action in this same cycle.
    if (_recovery.failed()) throw std::runtime_error("Recovery activation failed");
    const auto rawJump = commandJump(_recoveryBlendFrom, *_lowCmd, *_lowState);
    _applyRecoveryBlend(sample);
    const auto sentJump = commandJump(_recoveryBlendFrom, *_lowCmd, *_lowState);
    std::cout << "[LocoRecovery] -> RECOVERY: tilt=" << sample.tilt
              << " rad, outward_rate=" << sample.tiltRate << " rad/s, handoff="
              << _recoveryBlendDuration * 1000 << "ms, target_jump=" << sentJump.target
              << " (raw=" << rawJump.target << "), estimated_PD_torque_jump="
              << sentJump.torque << " (raw=" << rawJump.torque << ")" << std::endl;
}

double State_LocoRecovery::_recoveryBlendBudget(const Sample &sample) const
{
    if (_recoveryBlendMaxDuration == 0 || sample.tilt >= _hardTilt ||
        sample.tiltRate >= _recoveryBlendFastTiltRate) return 0;
    double budget = _recoveryBlendMaxDuration;
    if (sample.tiltRate > 0)
        budget = std::min(budget, _recoveryBlendTimeMargin *
            (_hardTilt - sample.tilt) / sample.tiltRate);
    // A one-cycle blend would reach alpha=1 on its first command anyway.
    return budget > _ctrlComp->dt + 1e-9 ? budget : 0;
}

void State_LocoRecovery::_applyRecoveryBlend(const Sample &sample)
{
    if (_recoveryBlendSeconds >= _recoveryBlendDuration) return;
    const double budget = _recoveryBlendBudget(sample);
    if (budget == 0) {
        _recoveryBlendDuration = _recoveryBlendSeconds;
        std::cout << "[LocoRecovery] Recovery handoff shortened: immediate takeover for rising risk"
                  << std::endl;
        return;
    }
    // Only shorten the deadline as risk rises. Never prolong a started handoff.
    _recoveryBlendDuration = std::min(_recoveryBlendDuration, _recoveryBlendSeconds + budget);
    _recoveryBlendSeconds = std::min(_recoveryBlendDuration, _recoveryBlendSeconds + _ctrlComp->dt);
    const double alpha = smoothstep(_recoveryBlendSeconds / _recoveryBlendDuration);
    blendCommand(_recoveryBlendFrom, *_lowCmd, alpha);
    if (_recoveryBlendSeconds >= _recoveryBlendDuration)
        std::cout << "[LocoRecovery] Recovery handoff complete" << std::endl;
}

void State_LocoRecovery::enter()
{
    _failed = _commandsArmed = _policyActive = false;
    _entryActionReady = false;
    _recoveryBlendSeconds = _recoveryBlendDuration = 0;
    _lastUserCommand = _lowState->userCmd;
    _riskSeconds = _stableSeconds = _recoverySeconds = _returnSeconds = _commandSeconds = 0;
    _phase = Phase::LOCO;
    _loco.setCommandGain(0);
    // A stop arriving while the outer FSM was changing states must win on entry.
    if (_lowState->userCmd == UserCommand::L2_B || _lowState->userCmd == UserCommand::SELECT) {
        _damping();
        return;
    }
    try {
        const auto sample = _readSample();
        if (sample.tilt >= _triggerTilt || !_standing(sample)) {
            _startRecovery(sample);
            _entryActionReady = true;
        } else {
            _policyActive = true;
            _loco.enter();
            std::cout << "[LocoRecovery] -> LOCO; center sticks to enable commands" << std::endl;
        }
    } catch (const std::exception &error) { _fail(error.what()); }
}

void State_LocoRecovery::_runLoco()
{
    if (_phase == Phase::RETURNING) {
        _loco.setCommandGain(0);
    } else if (!_commandsArmed) {
        const auto &stick = _lowState->userValue;
        _commandsArmed = std::abs(stick.lx) <= _stickDeadZone &&
                         std::abs(stick.ly) <= _stickDeadZone &&
                         std::abs(stick.rx) <= _stickDeadZone;
        _loco.setCommandGain(0);
        if (_commandsArmed)
            std::cout << "[LocoRecovery] Walking commands enabled after stick centering" << std::endl;
    } else {
        _commandSeconds = std::min(_commandRamp, _commandSeconds + _ctrlComp->dt);
        _loco.setCommandGain(static_cast<float>(_commandSeconds / _commandRamp));
    }
    _loco.run();
    if (_phase != Phase::RETURNING) return;
    _returnSeconds = std::min(_returnDuration, _returnSeconds + _ctrlComp->dt);
    const double t = _returnSeconds / _returnDuration;
    blendCommand(_returnFrom, *_lowCmd, smoothstep(t));
    if (_returnSeconds >= _returnDuration) {
        _phase = Phase::LOCO;
        std::cout << "[LocoRecovery] -> LOCO; center sticks to enable commands" << std::endl;
    }
}

void State_LocoRecovery::run()
{
    if (_failed || _lowState->userCmd == UserCommand::L2_B ||
        _lowState->userCmd == UserCommand::SELECT) {
        _damping();
        return;
    }
    try {
        const auto sample = _readSample();
        if (_entryActionReady) {
            _entryActionReady = false;
            return; // enterAutomatic already inferred on this entry sample.
        }
        if (_phase != Phase::RECOVERY && _falling(sample)) {
            _startRecovery(sample);
        } else if (_phase == Phase::RECOVERY) {
            _recovery.run();
            if (_recovery.failed()) throw std::runtime_error("Recovery policy failed");
            _applyRecoveryBlend(sample);
            _recoverySeconds += _ctrlComp->dt;
            _stableSeconds = _recoveryBlendSeconds >= _recoveryBlendDuration && _standing(sample) ?
                _stableSeconds + _ctrlComp->dt : 0;
            if (_recoverySeconds + 1e-9 >= _minimumRecovery &&
                _stableSeconds + 1e-9 >= _stableHold) {
                _returnFrom = *_lowCmd;
                _recovery.exit();
                _phase = Phase::RETURNING;
                _returnSeconds = _riskSeconds = 0;
                _loco.setCommandGain(0);
                _loco.enter(); // Resets LSTM, action and command histories.
                *_lowCmd = _returnFrom; // Start blending on the following sample.
                std::cout << "[LocoRecovery] -> RETURNING after stable stance" << std::endl;
            } else {
                _reportRecovery(sample);
                if (!_recoveryWarned && _recoverySeconds >= _recoveryWarningSeconds) {
                    _recoveryWarned = true;
                    std::cout << "[LocoRecovery] Recovery taking longer than " << _recoveryWarningSeconds
                              << "s; keeping Recovery active with joint support. "
                                 "L2+B stops; standing gate remains required for Loco." << std::endl;
                }
            }
        } else {
            _runLoco();
        }
        for (const auto &cmd : _lowCmd->motorCmd)
            if (!std::isfinite(cmd.q) || !std::isfinite(cmd.dq) || !std::isfinite(cmd.tau) ||
                !std::isfinite(cmd.Kp) || !std::isfinite(cmd.Kd))
                throw std::runtime_error("Non-finite composite policy command");
    } catch (const std::exception &error) { _fail(error.what()); }
}

void State_LocoRecovery::exit()
{
    if (_policyActive) {
        if (_phase == Phase::RECOVERY) _recovery.exit();
        else _loco.exit();
    }
    _policyActive = false;
    _recoveryBlendSeconds = _recoveryBlendDuration = 0;
    _loco.setCommandGain(1);
}

FSMStateName State_LocoRecovery::checkChange()
{
    const auto command = _lowState->userCmd;
    const bool fresh = command != _lastUserCommand;
    _lastUserCommand = command;
    if (command == UserCommand::SELECT) throw std::runtime_error("exit..");
    if (_failed || command == UserCommand::L2_B) return FSMStateName::PASSIVE;
    // Keep explicit access to the original manual recovery mode.
    if (fresh && command == UserCommand::R2_X) return FSMStateName::MJAMP_RECOVERY;
    // Do not let a held locomotion button bypass recovery or its return gate.
    if (fresh && _phase == Phase::LOCO) {
        if (command == UserCommand::R2_A) return FSMStateName::MJAMP;
        if (command == UserCommand::R1_UP) return FSMStateName::WBC;
    }
    return FSMStateName::LOCO_RECOVERY;
}
