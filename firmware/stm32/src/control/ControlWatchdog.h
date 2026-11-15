#pragma once

#include <cstdint>

#include "../safety/HealthState.h"

class ControlWatchdog
{
public:
    ControlWatchdog(
        uint32_t overrunThresholdUs,
        uint32_t lostThresholdUs,
        uint32_t maxConsecutiveOverruns);

    HealthState update(uint32_t elapsedUs);

    void reset();

private:
    uint32_t overrunThresholdUs_;
    uint32_t lostThresholdUs_;
    uint32_t maxConsecutiveOverruns_;

    uint32_t consecutiveOverruns_ = 0;
};
