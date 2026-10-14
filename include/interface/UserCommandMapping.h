#pragma once

#include "common/enumClass.h"
#include "common/gamepad.hpp"

// Decode the current held buttons. No button held must clear stale commands.
inline UserCommand userCommandFromGamepad(const unitree::common::Gamepad &pad,
                                          bool strictStandaloneR1 = false)
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
    if (pad.R1.pressed) {
        // Keep the original decoding for every existing task. Protection wait
        // must distinguish an unmapped combination from a physical release:
        // otherwise R1+A followed by releasing only A could start recovery.
        const bool otherHeld = pad.L1.pressed || pad.start.pressed || pad.select.pressed ||
            pad.R2.pressed || pad.L2.pressed || pad.F1.pressed || pad.F2.pressed ||
            pad.A.pressed || pad.B.pressed || pad.X.pressed || pad.Y.pressed ||
            pad.up.pressed || pad.right.pressed || pad.down.pressed || pad.left.pressed;
        return strictStandaloneR1 && otherHeld ? UserCommand::R1_OTHER : UserCommand::R1;
    }
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
