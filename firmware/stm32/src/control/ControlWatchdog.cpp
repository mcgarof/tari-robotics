#include "ControlWatchdog.h"

ControlWatchdog::ControlWatchdog(
    uint32_t overrunThresholdUs,
    uint32_t lostThresholdUs,
    uint32_t maxConsecutiveOverruns)
    : overrunThresholdUs_(overrunThresholdUs),
      lostThresholdUs_(lostThresholdUs),
      maxConsecutiveOverruns_(maxConsecutiveOverruns)
{
}

HealthState ControlWatchdog::update(uint32_t elapsedUs)
{
    if (elapsedUs > lostThresholdUs_)
    {
        consecutiveOverruns_ = 0;
        return HealthState::UNAVAILABLE;
    }

    if (elapsedUs > overrunThresholdUs_)
    {
        ++consecutiveOverruns_;

        if (consecutiveOverruns_ >= maxConsecutiveOverruns_)
            return HealthState::UNAVAILABLE;

        return HealthState::OK;
    }

    consecutiveOverruns_ = 0;
    return HealthState::OK;
}

void ControlWatchdog::reset()
{
    consecutiveOverruns_ = 0;
}
