#include "CommandSequenceTracker.h"

bool CommandSequenceTracker::accept(
    uint32_t sessionId,
    uint16_t sequence)
{
    // First valid command after startup/reset.
    if (!initialized_)
    {
        initialized_ = true;
        sessionId_ = sessionId;
        lastSequence_ = sequence;
        return true;
    }

    // A different session means that the sender restarted
    // or intentionally started a new command session.
    if (sessionId != sessionId_)
    {
        sessionId_ = sessionId;
        lastSequence_ = sequence;
        return true;
    }

    // Modular comparison. This also handles:
    //
    //   65534 -> 65535 -> 0 -> 1
    //
    // A positive signed difference means that the received
    // sequence is newer than the last accepted one.
    const int16_t difference =
        static_cast<int16_t>(
            static_cast<uint16_t>(
                sequence - lastSequence_));

    if (difference <= 0)
    {
        return false;
    }

    lastSequence_ = sequence;
    return true;
}

void CommandSequenceTracker::reset()
{
    initialized_ = false;
    sessionId_ = 0;
    lastSequence_ = 0;
}
