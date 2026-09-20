#pragma once

#include "common/enumClass.h"
#include "common/gamepad.hpp"

// Decode the current held buttons. No button held must clear stale commands.
inline UserCommand userCommandFromGamepad(const unitree::common::Gamepad &pad)
{
    if (pad.select.pressed) return UserCommand::SELECT;
    if (pad.L2.pressed && pad.B.pressed) return UserCommand::L2_B;
    if (pad.R2.pressed && pad.X.pressed) return UserCommand::R2_X;
    if (pad.R2.pressed && pad.Y.pressed) return UserCommand::R2_Y;
    if (pad.R2.pressed && pad.A.pressed) return UserCommand::R2_A;
    if (pad.R2.pressed && pad.B.pressed) return UserCommand::R2_B;
    if (pad.R2.pressed && pad.up.pressed) return UserCommand::R2_UP;
    if (pad.R2.pressed && pad.down.pressed) return UserCommand::R2_DOWN;
    if (pad.R1.pressed && pad.up.pressed) return UserCommand::R1_UP;
    if (pad.R1.pressed && pad.left.pressed) return UserCommand::R1_LEFT;
    if (pad.R1.pressed && pad.right.pressed) return UserCommand::R1_RIGHT;
    if (pad.start.pressed) return UserCommand::START;
    if (pad.R1.pressed) return UserCommand::R1;
    if (pad.L2.pressed) return UserCommand::L2;
    if (pad.R2.pressed) return UserCommand::R2;
    return UserCommand::NONE;
}

// Emit recovery entry once per observed press. A held entry key cannot re-enter
// recovery after a fault has returned the FSM to PASSIVE.
class RecoveryEntryLatch
{
public:
    UserCommand sample(UserCommand command)
    {
        const bool recovery = command == UserCommand::R2_X;
        const auto result = recovery && _held ? UserCommand::NONE : command;
        _held = recovery;
        return result;
    }
private:
    bool _held = false;
};
