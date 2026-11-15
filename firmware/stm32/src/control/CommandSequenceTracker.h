#pragma once

#include <cstdint>

class CommandSequenceTracker
{
public:
    bool accept(
        uint32_t sessionId,
        uint16_t sequence);

    void reset();

private:
    bool initialized_ = false;
    uint32_t sessionId_ = 0;
    uint16_t lastSequence_ = 0;
};
