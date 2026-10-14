#include "FSM/State_LocoRecovery2.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
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
}

State_LocoRecovery2::State_LocoRecovery2(CtrlComponents *ctrl, State_Loco &loco,
                                        State_MJAMP_RECOVERY &recovery)
    : FSMState(ctrl, FSMStateName::LOCO_RECOVERY2, "RL Loco+MJAmpRecovery2"),
      _loco(loco), _recovery(recovery)
{
    const auto cfg = config("loco_recovery2.json");
    _fallenTilt = positive(cfg, "fallen_tilt_rad");
    _hardFallenTilt = positive(cfg, "hard_fallen_tilt_rad");
    _fallenHold = positive(cfg, "fallen_hold_seconds");
    _protectionKd = cfg.at("protection_kd").get<double>();
    if (!std::isfinite(_protectionKd) || _protectionKd < 0 ||
        _protectionKd > std::numeric_limits<float>::max())
        throw std::runtime_error("Invalid protection_kd");
    _stableTilt = positive(cfg, "stable_tilt_rad");
    _stableAngularSpeed = positive(cfg, "stable_angular_velocity");
    _stableJointSpeed = positive(cfg, "stable_joint_velocity");
    _stableHold = positive(cfg, "stable_hold_seconds");
    _minimumRecovery = positive(cfg, "minimum_recovery_seconds");
    _recoveryWarningSeconds = positive(cfg, "recovery_warning_seconds");
    _returnDuration = positive(cfg, "return_blend_seconds");
    _commandRamp = positive(cfg, "command_ramp_seconds");
    _stickDeadZone = positive(cfg, "stick_dead_zone");
    _passiveKd = positive(config("passive.json"), "passive_kds");
    _standingMin = cfg.at("standing_joint_min").get<std::array<double, 12>>();
    _standingMax = cfg.at("standing_joint_max").get<std::array<double, 12>>();
    for (int i = 0; i < 12; ++i)
        if (!std::isfinite(_standingMin[i]) || !std::isfinite(_standingMax[i]) ||
            _standingMin[i] >= _standingMax[i])
            throw std::runtime_error("Invalid standing joint range");
    // Intercept a clear fall before another Loco inference can hit its ordinary
    // orientation guard. Equality permits the default 60 degree hard threshold.
    const double locoLimit = positive(config("loco.json"), "safe_projgravity_threshold");
    if (!(_stableTilt < _fallenTilt && _fallenTilt < _hardFallenTilt &&
          _hardFallenTilt < 1.57079632679 &&
          1.0 - std::cos(_hardFallenTilt) <= locoLimit + 1e-9) ||
        _stickDeadZone >= 1 || _recoveryWarningSeconds <= _minimumRecovery + _stableHold)
        throw std::runtime_error("Invalid Loco recovery2 hysteresis/timing configuration");
}

State_LocoRecovery2::Sample State_LocoRecovery2::_readSample() const
{
    if (!std::isfinite(_ctrlComp->dt) || std::abs(_ctrlComp->dt - 0.02) > 1e-6)
        throw std::runtime_error("Loco recovery2 requires dt = 0.02 s");
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
    return {tilt, std::sqrt(speed2)};
}

bool State_LocoRecovery2::_fallen(const Sample &sample)
{
    if (sample.tilt >= _hardFallenTilt) return true;
    _riskSeconds = sample.tilt >= _fallenTilt ? _riskSeconds + _ctrlComp->dt : 0;
    return _riskSeconds + 1e-9 >= _fallenHold;
}

bool State_LocoRecovery2::_standing(const Sample &sample) const
{
    if (sample.tilt > _stableTilt || sample.angularSpeed > _stableAngularSpeed) return false;
    for (int i = 0; i < 12; ++i) {
        const auto &joint = _lowState->motorState[i];
        if (joint.q < _standingMin[i] || joint.q > _standingMax[i] ||
            std::abs(joint.dq) > _stableJointSpeed) return false;
    }
    return true;
}

void State_LocoRecovery2::_reportRecovery(const Sample &sample)
{
    if (_recoverySeconds + 1e-9 < _nextRecoveryReport) return;
    _nextRecoveryReport = _recoverySeconds + 2.0;
    std::cout << "[LocoRecovery2] RECOVERY t=" << _recoverySeconds
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

void State_LocoRecovery2::_protection()
{
    for (auto &cmd : _lowCmd->motorCmd) {
        cmd.mode = 10;
        cmd.q = cmd.dq = cmd.tau = cmd.Kp = 0;
        cmd.Kd = _protectionKd;
    }
}

void State_LocoRecovery2::_damping()
{
    for (auto &cmd : _lowCmd->motorCmd) {
        cmd.mode = 10;
        cmd.q = cmd.dq = cmd.tau = cmd.Kp = 0;
        cmd.Kd = _passiveKd;
    }
}

void State_LocoRecovery2::_fail(const std::string &reason)
{
    if (!_failed) std::cerr << "[LocoRecovery2] " << reason << "; PASSIVE" << std::endl;
    _failed = true;
    _damping();
}

void State_LocoRecovery2::_enterProtection(const Sample &sample)
{
    if (_policyActive) {
        if (_phase == Phase::RECOVERY) _recovery.exit();
        else _loco.exit();
    }
    _policyActive = false;
    _phase = Phase::PROTECT_WAIT;
    _riskSeconds = _stableSeconds = _recoverySeconds = _returnSeconds = 0;
    _commandsArmed = false;
    _commandSeconds = 0;
    _loco.setCommandGain(0);
    _activationReleased = false;
    _lastProtectionCommand = _lowState->userCmd;
    _protection();
    std::cout << "[LocoRecovery2] -> PROTECT_WAIT: tilt=" << sample.tilt
              << " rad, Kp=0, Kd=" << _protectionKd
              << "; no policy inference. Release buttons, then press R1 to recover." << std::endl;
}

void State_LocoRecovery2::_runProtection(const Sample &sample)
{
    _protection(); // Waiting never reuses the last locomotion or recovery target.
    const auto command = _lowState->userCmd;
    const bool fresh = command != _lastProtectionCommand;
    _lastProtectionCommand = command;
    if (_lowState->heldUserCmd == UserCommand::NONE && command == UserCommand::NONE)
        _activationReleased = true;
    // Accept the physical standalone button as well as its logical event. A held
    // pre-fall button, a stale event, and R1+direction combinations cannot start.
    if (_activationReleased && fresh && command == UserCommand::R1 &&
        _lowState->heldUserCmd == UserCommand::R1)
        _startRecovery(sample);
    else if (command != UserCommand::NONE || _lowState->heldUserCmd != UserCommand::NONE)
        _activationReleased = false; // A rejected combination consumes this press too.
}

void State_LocoRecovery2::_startRecovery(const Sample &sample)
{
    _phase = Phase::RECOVERY;
    _policyActive = true;
    _activationReleased = false;
    _riskSeconds = _stableSeconds = _recoverySeconds = 0;
    _nextRecoveryReport = 2.0;
    _recoveryWarned = false;
    _commandsArmed = false;
    _commandSeconds = 0;
    // Recovery initializes all history from this activation sample. The unloaded
    // waiting command must not be blended into the support needed for get-up.
    _recovery.enterAutomatic();
    if (_recovery.failed()) throw std::runtime_error("Recovery activation failed");
    std::cout << "[LocoRecovery2] -> RECOVERY: R1 pressed, current tilt="
              << sample.tilt << " rad; velocity command held at zero" << std::endl;
}

void State_LocoRecovery2::enter()
{
    _failed = _commandsArmed = _policyActive = _activationReleased = false;
    _entryProtectionReady = false;
    _recoveryWarned = false;
    _lastUserCommand = _lastProtectionCommand = _lowState->userCmd;
    _riskSeconds = _stableSeconds = _recoverySeconds = _returnSeconds = _commandSeconds = 0;
    _phase = Phase::LOCO;
    _loco.setCommandGain(0);
    if (_lowState->userCmd == UserCommand::L2_B || _lowState->userCmd == UserCommand::SELECT) {
        _damping();
        return;
    }
    try {
        const auto sample = _readSample();
        if (!_standing(sample)) {
            _enterProtection(sample);
            _entryProtectionReady = true;
        } else {
            _policyActive = true;
            _loco.enter();
            std::cout << "[LocoRecovery2] -> LOCO; center sticks to enable commands" << std::endl;
        }
    } catch (const std::exception &error) { _fail(error.what()); }
}

void State_LocoRecovery2::_runLoco()
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
            std::cout << "[LocoRecovery2] Walking commands enabled after stick centering" << std::endl;
    } else {
        _commandSeconds = std::min(_commandRamp, _commandSeconds + _ctrlComp->dt);
        _loco.setCommandGain(static_cast<float>(_commandSeconds / _commandRamp));
    }
    _loco.run();
    if (_phase != Phase::RETURNING) return;
    _returnSeconds = std::min(_returnDuration, _returnSeconds + _ctrlComp->dt);
    const double t = _returnSeconds / _returnDuration;
    blendCommand(_returnFrom, *_lowCmd, t * t * (3.0 - 2.0 * t));
    if (_returnSeconds >= _returnDuration) {
        _phase = Phase::LOCO;
        std::cout << "[LocoRecovery2] -> LOCO; center sticks to enable commands" << std::endl;
    }
}

void State_LocoRecovery2::run()
{
    if (_failed || _lowState->userCmd == UserCommand::L2_B ||
        _lowState->userCmd == UserCommand::SELECT) {
        _damping();
        return;
    }
    try {
        const auto sample = _readSample();
        if (_entryProtectionReady) {
            _entryProtectionReady = false;
            _protection();
            return; // Outer FSM calls enter() and run() on the same feedback sample.
        }
        if ((_phase == Phase::LOCO || _phase == Phase::RETURNING) && _fallen(sample)) {
            _enterProtection(sample);
        } else if (_phase == Phase::PROTECT_WAIT) {
            _runProtection(sample);
        } else if (_phase == Phase::RECOVERY) {
            _recovery.run();
            if (_recovery.failed()) throw std::runtime_error("Recovery policy failed");
            _recoverySeconds += _ctrlComp->dt;
            _stableSeconds = _standing(sample) ? _stableSeconds + _ctrlComp->dt : 0;
            if (_recoverySeconds + 1e-9 >= _minimumRecovery &&
                _stableSeconds + 1e-9 >= _stableHold) {
                _returnFrom = *_lowCmd;
                _recovery.exit();
                _phase = Phase::RETURNING;
                _returnSeconds = _riskSeconds = 0;
                _loco.setCommandGain(0);
                _loco.enter();
                *_lowCmd = _returnFrom;
                std::cout << "[LocoRecovery2] -> RETURNING after stable stance" << std::endl;
            } else {
                _reportRecovery(sample);
                if (!_recoveryWarned && _recoverySeconds >= _recoveryWarningSeconds) {
                    _recoveryWarned = true;
                    std::cout << "[LocoRecovery2] Recovery taking longer than " << _recoveryWarningSeconds
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

void State_LocoRecovery2::exit()
{
    if (_policyActive) {
        if (_phase == Phase::RECOVERY) _recovery.exit();
        else _loco.exit();
    }
    _policyActive = _activationReleased = false;
    _entryProtectionReady = false;
    _loco.setCommandGain(1);
}

FSMStateName State_LocoRecovery2::checkChange()
{
    const auto command = _lowState->userCmd;
    const bool fresh = command != _lastUserCommand;
    _lastUserCommand = command;
    if (command == UserCommand::SELECT) throw std::runtime_error("exit..");
    if (_failed || command == UserCommand::L2_B) return FSMStateName::PASSIVE;
    // Protection cannot be bypassed by selecting another motion policy. Manual
    // mode switching remains available from ordinary locomotion only.
    if (fresh && _phase == Phase::LOCO) {
        if (command == UserCommand::R2_X) return FSMStateName::MJAMP_RECOVERY;
        if (command == UserCommand::R2_A) return FSMStateName::MJAMP;
        if (command == UserCommand::R1_UP) return FSMStateName::WBC;
    }
    return FSMStateName::LOCO_RECOVERY2;
}
