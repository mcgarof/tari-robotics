#pragma once

#include "CommandSequenceTracker.h"
#include "MotionCommandProtocol.h"
#include "MotionCommandStreamParser.h"
#include "MotionCommandValidator.h"

#include <cstdint>

class MotionCommandReceiver
{
public:
    explicit MotionCommandReceiver(
        const MotionCommandLimits& limits);

    bool pushByte(
        uint8_t byte,
        MotionCommandProtocol::Packet& packet);

    void reset();

private:
    MotionCommandStreamParser parser_;
    MotionCommandValidator validator_;
    CommandSequenceTracker sequenceTracker_;
};
