#include "SafetyInputs.h"

void SafetyInputs::update(
    bool enableLevel,
    bool resetLevel,
    uint32_t nowMs)
{
    // ENABLE:
    // - HIGH -> LOW is immediate (fail-safe).
    // - LOW -> HIGH must remain stable for the debounce interval.
    if (!enableLevel)
    {
        enableRequested_ = false;
        enableHighPending_ = false;
    }
    else if (!enableRequested_)
    {
        if (!enableHighPending_)
        {
            enableHighPending_ = true;
            enableHighSinceMs_ = nowMs;
        }
        else if (nowMs - enableHighSinceMs_ >= kEnableDebounceMs)
        {
            enableRequested_ = true;
            enableHighPending_ = false;
        }
    }

    // RESET:
    // - HIGH must remain stable for the debounce interval.
    // - Generate exactly one request when the press is accepted.
    // - No further request is generated until the input returns LOW.
    resetRequested_ = false;

    if (!resetLevel)
    {
        resetHighPending_ = false;
        resetAccepted_ = false;
    }
    else if (!resetAccepted_)
    {
        if (!resetHighPending_)
        {
            resetHighPending_ = true;
            resetHighSinceMs_ = nowMs;
        }
        else if (nowMs - resetHighSinceMs_ >= kResetDebounceMs)
        {
            resetRequested_ = true;
            resetAccepted_ = true;
            resetHighPending_ = false;
        }
    }
}

bool SafetyInputs::enableRequested() const
{
    return enableRequested_;
}

bool SafetyInputs::resetRequested() const
{
    return resetRequested_;
}
