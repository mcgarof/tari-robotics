#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

template <typename Sample, std::size_t Capacity>
class FlightRecorder
{
public:
    void record(uint32_t timestampUs, Sample sample)
    {
        if (frozen_)
            return;

        sample.timestampUs = timestampUs;

        samples_[head_] = sample;

        head_ = (head_ + 1) % Capacity;

        if (size_ < Capacity)
            ++size_;
    }

    std::size_t size() const
    {
        return size_;
    }

    constexpr std::size_t capacity() const
    {
        return Capacity;
    }

    bool empty() const
    {
        return size_ == 0;
    }

    bool get(std::size_t index, Sample& sample) const
    {
        if (index >= size_)
            return false;

        const std::size_t oldest =
            (head_ + Capacity - size_) % Capacity;

        const std::size_t physicalIndex =
            (oldest + index) % Capacity;

        sample = samples_[physicalIndex];
        return true;
    }

    void clear()
    {
        head_ = 0;
        size_ = 0;
    }

    void freeze()
    {
        frozen_ = true;
    }

    void unfreeze()
    {
        frozen_ = false;
    }

    void rearm()
    {
        clear();
        unfreeze();
    }

    bool frozen() const
    {
        return frozen_;
    }

private:
    std::array<Sample, Capacity> samples_{};

    std::size_t head_ = 0;
    std::size_t size_ = 0;

    bool frozen_ = false;
};
