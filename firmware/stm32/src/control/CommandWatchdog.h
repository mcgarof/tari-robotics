#pragma once

#include <cstdint>

class CommandWatchdog
{
public:
    explicit CommandWatchdog(uint32_t timeoutMs);

    void notifyCommandReceived(uint32_t nowMs);

    bool commandFresh(uint32_t nowMs) const;
    bool commandReceived() const;

    void reset();

private:
    uint32_t timeoutMs_;

    uint32_t lastCommandMs_ = 0;
    bool commandReceived_ = false;
};
