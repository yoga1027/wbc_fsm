#pragma once

#include "FSM/State_Loco.h"
#include "FSM/State_MJAmpRecovery.h"

// Owns switching, not policy sessions. The ordinary states are inactive while
// this state borrows them; only one policy produces the command each cycle.
class State_LocoRecovery : public FSMState
{
public:
    enum class Phase { LOCO, RECOVERY, RETURNING };
    State_LocoRecovery(CtrlComponents *ctrl, State_Loco &loco,
                       State_MJAMP_RECOVERY &recovery);
    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;
    Phase phase() const { return _phase; }

private:
    struct Sample { double tilt, tiltRate, angularSpeed; };
    State_Loco &_loco;
    State_MJAMP_RECOVERY &_recovery;
    Phase _phase = Phase::LOCO;
    bool _failed = false, _commandsArmed = false, _policyActive = false;
    bool _entryActionReady = false;
    UserCommand _lastUserCommand = UserCommand::NONE;
    double _riskSeconds = 0, _stableSeconds = 0, _recoverySeconds = 0;
    double _returnSeconds = 0, _commandSeconds = 0;
    double _recoveryBlendSeconds = 0, _recoveryBlendDuration = 0;
    double _nextRecoveryReport = 0;
    bool _recoveryWarned = false;
    double _triggerTilt, _hardTilt, _triggerRate, _triggerHold;
    double _stableTilt, _stableAngularSpeed, _stableJointSpeed, _stableHold;
    double _minimumRecovery, _recoveryWarningSeconds, _returnDuration, _commandRamp;
    double _recoveryBlendMaxDuration, _recoveryBlendFastTiltRate, _recoveryBlendTimeMargin;
    double _stickDeadZone, _passiveKd;
    std::array<double, 12> _standingMin{}, _standingMax{};
    LowlevelCmd _returnFrom;
    LowlevelCmd _recoveryBlendFrom;

    Sample _readSample() const;
    bool _falling(const Sample &sample);
    bool _standing(const Sample &sample) const;
    void _reportRecovery(const Sample &sample);
    void _startRecovery(const Sample &sample);
    double _recoveryBlendBudget(const Sample &sample) const;
    void _applyRecoveryBlend(const Sample &sample);
    void _runLoco();
    void _damping();
    void _fail(const std::string &reason);
};
