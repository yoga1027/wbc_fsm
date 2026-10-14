#pragma once

#include "FSM/State_Loco.h"
#include "FSM/State_MJAmpRecovery.h"

// Borrows the existing policy states, with an explicit unloaded wait between
// locomotion and recovery. No policy inference runs in PROTECT_WAIT.
class State_LocoRecovery2 : public FSMState
{
public:
    enum class Phase { LOCO, PROTECT_WAIT, RECOVERY, RETURNING };
    State_LocoRecovery2(CtrlComponents *ctrl, State_Loco &loco,
                        State_MJAMP_RECOVERY &recovery);
    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;
    Phase phase() const { return _phase; }

private:
    struct Sample { double tilt, angularSpeed; };
    State_Loco &_loco;
    State_MJAMP_RECOVERY &_recovery;
    Phase _phase = Phase::LOCO;
    bool _failed = false, _commandsArmed = false, _policyActive = false;
    bool _activationReleased = false, _recoveryWarned = false;
    bool _entryProtectionReady = false;
    UserCommand _lastUserCommand = UserCommand::NONE;
    UserCommand _lastProtectionCommand = UserCommand::NONE;
    double _riskSeconds = 0, _stableSeconds = 0, _recoverySeconds = 0;
    double _returnSeconds = 0, _commandSeconds = 0, _nextRecoveryReport = 0;
    double _fallenTilt, _hardFallenTilt, _fallenHold;
    double _stableTilt, _stableAngularSpeed, _stableJointSpeed, _stableHold;
    double _minimumRecovery, _recoveryWarningSeconds, _returnDuration, _commandRamp;
    double _stickDeadZone, _passiveKd, _protectionKd;
    std::array<double, 12> _standingMin{}, _standingMax{};
    LowlevelCmd _returnFrom;

    Sample _readSample() const;
    bool _fallen(const Sample &sample);
    bool _standing(const Sample &sample) const;
    void _reportRecovery(const Sample &sample);
    void _enterProtection(const Sample &sample);
    void _runProtection(const Sample &sample);
    void _startRecovery(const Sample &sample);
    void _runLoco();
    void _protection();
    void _damping();
    void _fail(const std::string &reason);
};
