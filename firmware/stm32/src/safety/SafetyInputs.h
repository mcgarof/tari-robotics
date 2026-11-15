#pragma once

#include <cstdint>

class SafetyInputs
{
public:
    void update(bool enableLevel, bool resetLevel, uint32_t nowMs);

    bool enableRequested() const;
    bool resetRequested() const;

private:
    static constexpr uint32_t kEnableDebounceMs = 30;
    static constexpr uint32_t kResetDebounceMs = 30;

    bool enableRequested_ = false;
    bool resetRequested_ = false;

    bool enableHighPending_ = false;
    uint32_t enableHighSinceMs_ = 0;

    bool resetHighPending_ = false;
    bool resetAccepted_ = false;
    uint32_t resetHighSinceMs_ = 0;
};
