#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "observability/Event.h"

template <std::size_t Capacity>
class EventRecorder {
    static_assert(Capacity > 0, "EventRecorder capacity must be greater than zero");

public:
    void record(
        uint32_t timestampUs,
        EventId id,
        EventSeverity severity)
    {
        events_[head_] = {
            timestampUs,
            nextSequence_++,
            id,
            severity
        };

        head_ = (head_ + 1) % Capacity;

        if (size_ < Capacity) {
            ++size_;
        }
    }

    std::size_t size() const {
        return size_;
    }

    constexpr std::size_t capacity() const {
        return Capacity;
    }

    bool empty() const {
        return size_ == 0;
    }

    bool get(std::size_t index, Event& event) const {
        if (index >= size_) {
            return false;
        }

        const std::size_t oldest =
            (head_ + Capacity - size_) % Capacity;

        event = events_[(oldest + index) % Capacity];
        return true;
    }

    void clear() {
        head_ = 0;
        size_ = 0;
    }

private:
    std::array<Event, Capacity> events_{};
    std::size_t head_ = 0;
    std::size_t size_ = 0;
    uint32_t nextSequence_ = 0;
};
