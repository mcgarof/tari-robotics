#pragma once

#include <cstddef>

#include "observability/FlightRecorder.h"
#include "observability/FlightRecorderPublisher.h"
#include "observability/NonBlockingWriter.h"

template <typename Sample, std::size_t Capacity>
class FlightDumpPublisher
{
public:
    using Serializer = std::size_t (*)(
        const Sample&,
        char*,
        std::size_t);

    explicit FlightDumpPublisher(Serializer serializer)
        : serializer_(serializer)
    {
    }

    void reset()
    {
        publisher_.reset();
        writer_.reset();
        active_ = false;
    }

    bool start()
    {
        if (active_)
            return false;

        publisher_.reset();
        writer_.reset();
        active_ = true;

        return true;
    }

    bool active() const
    {
        return active_;
    }

    template <typename Transport>
    void update(
        const FlightRecorder<Sample, Capacity>& recorder,
        Transport& transport)
    {
        if (!active_)
            return;

        // A serialized sample is currently being transmitted.
        if (writer_.busy())
        {
            writer_.update(transport);
            return;
        }

        // The previous sample has now been transmitted completely.
        if (writer_.complete())
        {
            publisher_.markPublished();
            writer_.reset();
        }

        Sample sample{};

        // No more samples: dump completed.
        if (!publisher_.next(recorder, sample))
        {
            active_ = false;
            return;
        }

        const std::size_t length =
            serializer_(
                sample,
                buffer_,
                sizeof(buffer_));

        if (length == 0)
        {
            // Serialization failure: abort the dump rather
            // than silently skipping a sample.
            active_ = false;
            return;
        }

        if (!writer_.begin(buffer_, length))
        {
            active_ = false;
            return;
        }

        writer_.update(transport);
    }

private:
    static constexpr std::size_t kBufferSize = 160;

    Serializer serializer_;

    FlightRecorderPublisher publisher_;
    NonBlockingWriter writer_;

    char buffer_[kBufferSize]{};

    bool active_ = false;
};
