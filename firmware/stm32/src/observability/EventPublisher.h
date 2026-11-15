#pragma once

#include <cstddef>
#include <cstdint>

#include "observability/Event.h"
#include "observability/EventRecorder.h"

enum class EventPublishStatus {
    NO_EVENT,
    EVENT_AVAILABLE,
    EVENTS_LOST
};

inline bool sequenceBefore(uint32_t a, uint32_t b)
{
    return static_cast<int32_t>(a - b) < 0;
}

class EventPublisher
{
public:
    EventPublisher() = default;

    template <std::size_t Capacity>
    EventPublishStatus next(
        const EventRecorder<Capacity>& recorder,
        Event& event) const
    {
        if (recorder.empty())
            return EventPublishStatus::NO_EVENT;

        Event oldest{};

        if (!recorder.get(0, oldest))
            return EventPublishStatus::NO_EVENT;

        // The event expected by the publisher has already been overwritten.
        if (sequenceBefore(nextSequence_, oldest.sequence))
        {
            event = oldest;
            return EventPublishStatus::EVENTS_LOST;
        }

        const uint32_t offset =
            nextSequence_ - oldest.sequence;

        if (offset >= recorder.size())
            return EventPublishStatus::NO_EVENT;

        if (!recorder.get(offset, event))
            return EventPublishStatus::NO_EVENT;

        return EventPublishStatus::EVENT_AVAILABLE;
    }

    uint32_t nextSequence() const
    {
        return nextSequence_;
    }

    void markPublished(const Event& event)
    {
        nextSequence_ = event.sequence + 1;
    }

private:
    uint32_t nextSequence_ = 0;
};
