#pragma once

#include <cstddef>

#include "observability/FlightRecorder.h"

class FlightRecorderPublisher
{
public:
    FlightRecorderPublisher() = default;

    template <typename Sample, std::size_t Capacity>
    bool next(
        const FlightRecorder<Sample, Capacity>& recorder,
        Sample& sample) const
    {
        return recorder.get(index_, sample);
    }

    void markPublished()
    {
        ++index_;
    }

    void reset()
    {
        index_ = 0;
    }

private:
    std::size_t index_ = 0;
};
