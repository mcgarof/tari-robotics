#pragma once

#include "observability/FlightEncoding.h"
#include <cstddef>

constexpr std::size_t kFastFlightRecorderCapacity = 3200;

struct FastFlightSample
{
    uint32_t timestampUs;

    int16_t pitch;
    int16_t pitchRate;
    int16_t yaw;
    int16_t yawRate;

    int16_t leftWheelSpeed;
    int16_t rightWheelSpeed;

    int16_t leftTorque;
    int16_t rightTorque;
};

static_assert(
    sizeof(FastFlightSample) == 20,
    "FastFlightSample must be exactly 20 bytes");
