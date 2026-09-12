#pragma once

#include "FSM/FSMState.h"
#include <onnxruntime_cxx_api.h>
#include <array>
#include <memory>
#include <optional>

using namespace ArmatureConstants;

class State_MJAMP_RECOVERY : public FSMState
{
public:
    explicit State_MJAMP_RECOVERY(CtrlComponents *ctrlComp);
    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    static constexpr int kNumDof = 29;
    static constexpr int kFrameSize = 96;
    static constexpr int kHistoryLength = 4;
    static constexpr int kObsSize = kFrameSize * kHistoryLength;
    static constexpr float kDeadZone = 0.2f;

    Ort::Env _env{ORT_LOGGING_LEVEL_WARNING, "mjamp_recovery"};
    Ort::SessionOptions _sessionOptions;
    std::unique_ptr<Ort::Session> _session;
    std::string _modelPath;
    std::array<float, kObsSize> _observation{};
    std::array<float, kNumDof> _action{};
    std::array<float, 3> _command{};
    std::array<float, 3> _previousCommand{};
    std::array<float, 2> _vxLimit{}, _vxSlowLimit{}, _vyLimit{}, _yawLimit{};
    float _commandSmoothing = 0.0f;
    std::optional<float> _clipObservations, _clipActions;
    bool _enableOrientationExit = false;
    float _orientationExitAngle = 1.22f;
    float _switchMaxTilt = 0.34906585f;
    float _switchMaxAngularVelocity = 0.5f;
    float _switchHoldSeconds = 0.5f;
    double _stableSeconds = 0.0;
    float _passiveKd = 10.0f;
    bool _highSpeed = false;
    bool _commandArmed = false;
    bool _terminate = false;
    bool _firstRun = true;
    UserCommand _lastUserCommand = UserCommand::NONE;

    void _loadConfig();
    void _loadPolicy();
    void _updateCommand();
    std::array<float, kFrameSize> _readFrame();
    void _inferAndWriteCommand();
    void _writeDamping();
    void _fail(const std::string &reason);

    const float _default_dof_pos[kNumDof] = {-0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
                                        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
                                        0.0, 0.0, 0.0,
                                        0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
                                        0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,};

    const int dof_mapping_mj[kNumDof] = {0, 1, 2, 3, 4,5,
        6,7,8,9,10,11,
        12,13,14,
        15,16,17,18,19,20,21,
        22,23,24,25,26,27,28}; // motor order for mujoco

    double dof_action_scale[kNumDof]{};

    const double limit_dof_tau[kNumDof] = {
        Limit::LIMIT_7520_22, Limit::LIMIT_7520_22, Limit::LIMIT_7520_14, Limit::LIMIT_7520_22, 2.0 * Limit::LIMIT_5020, 2.0 * Limit::LIMIT_5020,
        Limit::LIMIT_7520_22, Limit::LIMIT_7520_22, Limit::LIMIT_7520_14, Limit::LIMIT_7520_22, 2.0 * Limit::LIMIT_5020, 2.0 * Limit::LIMIT_5020,
        Limit::LIMIT_7520_14, 2.0 * Limit::LIMIT_5020, 2.0 * Limit::LIMIT_5020,
        Limit::LIMIT_5020, Limit::LIMIT_5020, Limit::LIMIT_5020, Limit::LIMIT_5020, Limit::LIMIT_5020, Limit::LIMIT_5010_16, Limit::LIMIT_5010_16,
        Limit::LIMIT_5020, Limit::LIMIT_5020, Limit::LIMIT_5020, Limit::LIMIT_5020, Limit::LIMIT_5020, Limit::LIMIT_5010_16, Limit::LIMIT_5010_16,}; // 电机力矩限制

    const double dof_Kps[kNumDof] = {STIFFNESS_7520_22, STIFFNESS_7520_22, STIFFNESS_7520_14, STIFFNESS_7520_22, 2.0 * STIFFNESS_5020, 2.0 * STIFFNESS_5020,
                                STIFFNESS_7520_22, STIFFNESS_7520_22, STIFFNESS_7520_14, STIFFNESS_7520_22, 2.0 * STIFFNESS_5020, 2.0 * STIFFNESS_5020,
                                STIFFNESS_7520_14, 2.0 * STIFFNESS_5020, 2.0 * STIFFNESS_5020,
                                STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5010_16, STIFFNESS_5010_16,
                                STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5010_16, STIFFNESS_5010_16,}; // 电机Kp参数

    const double dof_Kds[kNumDof] = {DAMPING_7520_22, DAMPING_7520_22, DAMPING_7520_14, DAMPING_7520_22, 2.0 * DAMPING_5020, 2.0 * DAMPING_5020,
                                DAMPING_7520_22, DAMPING_7520_22, DAMPING_7520_14, DAMPING_7520_22, 2.0 * DAMPING_5020, 2.0 * DAMPING_5020,
                                DAMPING_7520_14, 2.0 * DAMPING_5020, 2.0 * DAMPING_5020,
                                DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5010_16, DAMPING_5010_16,
                                DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5010_16, DAMPING_5010_16,}; // 电机Kd参数
};
