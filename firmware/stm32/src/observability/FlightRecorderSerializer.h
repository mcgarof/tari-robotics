#pragma once

#include <cstddef>
#include <cstdio>

#include "observability/FastFlightSample.h"
#include "observability/SlowFlightSample.h"
#include "observability/Event.h"

constexpr unsigned kFlightFormatVersion = 1;

inline std::size_t serializeFastFlightSample(
    const FastFlightSample& sample,
    char* buffer,
    std::size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0)
        return 0;

    const int length = std::snprintf(
        buffer,
        bufferSize,
        "F,%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
        kFlightFormatVersion,
        static_cast<unsigned long>(sample.timestampUs),
        static_cast<int>(sample.pitch),
        static_cast<int>(sample.pitchRate),
        static_cast<int>(sample.yaw),
        static_cast<int>(sample.yawRate),
        static_cast<int>(sample.leftWheelSpeed),
        static_cast<int>(sample.rightWheelSpeed),
        static_cast<int>(sample.leftTorque),
        static_cast<int>(sample.rightTorque));

    if (length < 0)
        return 0;

    const auto required =
        static_cast<std::size_t>(length);

    if (required >= bufferSize)
        return 0;

    return required;
}

inline std::size_t serializeSlowFlightSample(
    const SlowFlightSample& sample,
    char* buffer,
    std::size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0)
        return 0;

    const int length = std::snprintf(
        buffer,
        bufferSize,
        "S,%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u\n",
        kFlightFormatVersion,
        static_cast<unsigned long>(sample.timestampUs),
        static_cast<int>(sample.pitch),
        static_cast<int>(sample.pitchRate),
        static_cast<int>(sample.yaw),
        static_cast<int>(sample.yawRate),
        static_cast<int>(sample.leftWheelSpeed),
        static_cast<int>(sample.rightWheelSpeed),
        static_cast<int>(sample.commandVelocity),
        static_cast<int>(sample.commandSteering),
        static_cast<int>(sample.speedIntegralTorque),
        static_cast<unsigned>(sample.robotMode),
        static_cast<unsigned>(sample.statusFlags));

    if (length < 0)
        return 0;

    const auto required =
        static_cast<std::size_t>(length);

    if (required >= bufferSize)
        return 0;

    return required;
}

inline std::size_t serializeEvent(
    const Event& event,
    char* buffer,
    std::size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0)
        return 0;

    const int length = std::snprintf(
        buffer,
        bufferSize,
        "E,%u,%lu,%lu,%u,%u\n",
        kFlightFormatVersion,
        static_cast<unsigned long>(event.timestampUs),
        static_cast<unsigned long>(event.sequence),
        static_cast<unsigned>(event.id),
        static_cast<unsigned>(event.severity));

    if (length < 0)
        return 0;

    const auto required =
        static_cast<std::size_t>(length);

    if (required >= bufferSize)
        return 0;

    return required;
}

inline std::size_t serializeEventsLost(
    uint32_t expectedSequence,
    uint32_t oldestAvailableSequence,
    char* buffer,
    std::size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0)
        return 0;

    const int length = std::snprintf(
        buffer,
        bufferSize,
        "L,%u,%lu,%lu\n",
        kFlightFormatVersion,
        static_cast<unsigned long>(expectedSequence),
        static_cast<unsigned long>(oldestAvailableSequence));

    if (length < 0)
        return 0;

    const auto required =
        static_cast<std::size_t>(length);

    if (required >= bufferSize)
        return 0;

    return required;
}
