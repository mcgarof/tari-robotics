#include "MotionCommandReceiver.h"

MotionCommandReceiver::MotionCommandReceiver(
    const MotionCommandLimits& limits)
    : validator_(limits)
{
}

bool MotionCommandReceiver::pushByte(
    uint8_t byte,
    MotionCommandProtocol::Packet& packet)
{
    MotionCommandProtocol::Packet candidate{};

    if (!parser_.pushByte(byte, candidate))
    {
        return false;
    }

    if (!validator_.valid(candidate))
    {
        return false;
    }

    if (!sequenceTracker_.accept(
            candidate.sessionId,
            candidate.sequence))
    {
        return false;
    }

    packet = candidate;
    return true;
}

void MotionCommandReceiver::reset()
{
    parser_.reset();
    sequenceTracker_.reset();
}
