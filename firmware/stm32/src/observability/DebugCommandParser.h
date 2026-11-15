#pragma once

#include <cstddef>
#include <cstring>

enum class DebugCommand
{
    NONE,
    DUMP_EVENTS,
    DUMP_FAST,
    DUMP_SLOW,
    CLEAR_FLIGHT,
    REARM_FLIGHT,
    GET_PARAMS
};

class DebugCommandParser
{
public:
    DebugCommand push(char c)
    {
        if (c == '\r')
            return DebugCommand::NONE;

        if (c == '\n')
        {
            if (overflow_)
            {
                reset();
                return DebugCommand::NONE;
            }

            buffer_[length_] = '\0';

            DebugCommand command = DebugCommand::NONE;

            if (std::strcmp(buffer_, "DUMP EVENTS") == 0)
                command = DebugCommand::DUMP_EVENTS;
            else if (std::strcmp(buffer_, "DUMP FAST") == 0)
                command = DebugCommand::DUMP_FAST;
            else if (std::strcmp(buffer_, "DUMP SLOW") == 0)
                command = DebugCommand::DUMP_SLOW;
            else if (std::strcmp(buffer_, "CLEAR FLIGHT") == 0)
                command = DebugCommand::CLEAR_FLIGHT;
            else if (std::strcmp(buffer_, "REARM FLIGHT") == 0)
                command = DebugCommand::REARM_FLIGHT;
            else if (std::strcmp(buffer_, "GET_PARAMS") == 0)
                command = DebugCommand::GET_PARAMS;

            reset();
            return command;
        }

        if (length_ >= kBufferSize - 1)
        {
            reset();
            overflow_ = true;
            return DebugCommand::NONE;
        }

        // Once a command has overflowed, discard everything
        // until the end of the line.
        if (overflow_)
            return DebugCommand::NONE;

        buffer_[length_++] = c;

        return DebugCommand::NONE;
    }

    void reset()
    {
        length_ = 0;
        overflow_ = false;
        buffer_[0] = '\0';
    }

private:
    static constexpr std::size_t kBufferSize = 32;

    char buffer_[kBufferSize]{};
    std::size_t length_ = 0;
    bool overflow_ = false;
};
