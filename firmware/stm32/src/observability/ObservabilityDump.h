#pragma once

enum class ObservabilityDumpState
{
    IDLE,
    EVENTS,
    FAST,
    SLOW
};

class ObservabilityDump
{
public:
    ObservabilityDumpState state() const
    {
        return state_;
    }

    bool active() const
    {
        return state_ != ObservabilityDumpState::IDLE;
    }

    bool start(ObservabilityDumpState state)
    {
        if (state == ObservabilityDumpState::IDLE ||
            active())
        {
            return false;
        }

        state_ = state;
        return true;
    }

    void stop()
    {
        state_ = ObservabilityDumpState::IDLE;
    }

private:
    ObservabilityDumpState state_ =
        ObservabilityDumpState::IDLE;
};
