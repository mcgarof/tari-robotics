#pragma once

#include <cstddef>

#include "observability/Event.h"
#include "observability/EventPublisher.h"
#include "observability/EventRecorder.h"
#include "observability/FlightRecorderSerializer.h"
#include "observability/NonBlockingWriter.h"

template <std::size_t Capacity>
class EventDumpPublisher
{
public:
    using Serializer = std::size_t (*)(
        const Event&,
        char*,
        std::size_t);

    explicit EventDumpPublisher(Serializer serializer)
        : serializer_(serializer)
    {
    }

    bool start()
    {
        if (active_)
            return false;

        writer_.reset();
        pendingOutput_ = PendingOutput::NONE;
        publishEventAfterLoss_ = false;
        active_ = true;

        return true;
    }

    void reset()
    {
        writer_.reset();
        pendingOutput_ = PendingOutput::NONE;
        publishEventAfterLoss_ = false;
        active_ = false;
    }

    bool active() const
    {
        return active_;
    }

    template <typename Transport>
    void update(
        const EventRecorder<Capacity>& recorder,
        Transport& transport)
    {
        if (!active_)
            return;

        if (writer_.busy())
        {
            writer_.update(transport);
            return;
        }

        if (writer_.complete())
        {
            writer_.reset();

            if (pendingOutput_ == PendingOutput::EVENT)
            {
                publisher_.markPublished(currentEvent_);
            }
            else if (
                pendingOutput_ == PendingOutput::EVENTS_LOST &&
                publishEventAfterLoss_)
            {
                const std::size_t length =
                    serializer_(
                        currentEvent_,
                        buffer_,
                        sizeof(buffer_));

                if (length == 0 ||
                    !writer_.begin(buffer_, length))
                {
                    active_ = false;
                    return;
                }

                pendingOutput_ = PendingOutput::EVENT;
                publishEventAfterLoss_ = false;

                writer_.update(transport);
                return;
            }

            pendingOutput_ = PendingOutput::NONE;
        }

        Event event{};

        const uint32_t expectedSequence =
            publisher_.nextSequence();

        const EventPublishStatus status =
            publisher_.next(recorder, event);

        if (status == EventPublishStatus::NO_EVENT)
        {
            active_ = false;
            return;
        }

        currentEvent_ = event;

        std::size_t length = 0;

        if (status == EventPublishStatus::EVENTS_LOST)
        {
            length = serializeEventsLost(
                expectedSequence,
                currentEvent_.sequence,
                buffer_,
                sizeof(buffer_));

            pendingOutput_ = PendingOutput::EVENTS_LOST;
            publishEventAfterLoss_ = true;
        }
        else
        {
            length = serializer_(
                currentEvent_,
                buffer_,
                sizeof(buffer_));

            pendingOutput_ = PendingOutput::EVENT;
        }

        if (length == 0 ||
            !writer_.begin(buffer_, length))
        {
            active_ = false;
            return;
        }

        writer_.update(transport);
    }

private:
    enum class PendingOutput
    {
        NONE,
        EVENTS_LOST,
        EVENT
    };

    static constexpr std::size_t kBufferSize = 96;

    Serializer serializer_;

    EventPublisher publisher_;
    NonBlockingWriter writer_;

    Event currentEvent_{};
    char buffer_[kBufferSize]{};

    PendingOutput pendingOutput_ = PendingOutput::NONE;
    bool publishEventAfterLoss_ = false;

    bool active_ = false;
};
