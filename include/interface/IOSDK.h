#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include <string>
#include "common/gamepad.hpp"
#include "interface/UserCommandMapping.h"
#include "interface/StartupOptions.h"
#include <chrono>
#include <functional>
#include <mutex>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

const int G1_NUM_MOTOR = 29;
enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

class IOSDK : public IOInterface
{
private:
    ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
    LowlevelCmd _lowCmd;
    LowlevelState _lowState;
    REMOTE_DATA_RX rx_;
    Gamepad gamepad_;
    uint8_t mode_machine_ = 0;
    UserCommand userCmd_ = UserCommand::NONE;
    RecoveryEntryLatch recoveryEntryLatch_;
    bool strictStandaloneR1_ = false;
    UserValue userValue_;
    std::mutex stateMutex_;
    bool stateReady_ = false;
    bool controlReady_ = false;
    // Same lifetime as the interface, as in the BeiChe implementation.
    std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> motionClient_;
    std::chrono::steady_clock::time_point lastStateTime_{};
    // Destroy subscription before anything its callback accesses.
    ChannelSubscriberPtr<LowState_> lowstate_subscriber_;

    void LowStateHandler(const void *message);
    bool hasFreshState();

public:
    explicit IOSDK(const StartupOptions &options,
                   const std::function<bool()> &cancelled = [] { return false; });
    ~IOSDK() override;
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
};






#endif //IOSDK_H
