#include "CommandWatchdog.h"

CommandWatchdog::CommandWatchdog(uint32_t timeoutMs)
    : timeoutMs_(timeoutMs)
{
}

void CommandWatchdog::notifyCommandReceived(uint32_t nowMs)
{
    lastCommandMs_ = nowMs;
    commandReceived_ = true;
}

bool CommandWatchdog::commandFresh(uint32_t nowMs) const
{
    if (!commandReceived_)
    {
        return false;
    }

    return static_cast<uint32_t>(
        nowMs - lastCommandMs_) <= timeoutMs_;
}

bool CommandWatchdog::commandReceived() const
{
    return commandReceived_;
}

void CommandWatchdog::reset()
{
    lastCommandMs_ = 0;
    commandReceived_ = false;
}
