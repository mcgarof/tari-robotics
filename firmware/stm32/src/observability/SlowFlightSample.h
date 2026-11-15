#pragma once

#include "observability/FlightEncoding.h"
#include <cstddef>

constexpr std::size_t kSlowFlightRecorderCapacity = 1300;

enum SlowFlightStatusFlag : uint8_t
{
    SLOW_IMU_OK            = 1u << 0,
    SLOW_MOTOR_OK          = 1u << 1,
    SLOW_WHEEL_FEEDBACK_OK = 1u << 2,
    SLOW_CONTROL_OK        = 1u << 3,
    SLOW_COMMAND_FRESH     = 1u << 4,
    SLOW_TILT_EXCEEDED     = 1u << 5,
    SLOW_ENABLE_REQUESTED  = 1u << 6,
    SLOW_RESET_REQUESTED   = 1u << 7
};

struct SlowFlightSample
{
    uint32_t timestampUs;

    int16_t pitch;
    int16_t pitchRate;
    int16_t yaw;
    int16_t yawRate;

    int16_t leftWheelSpeed;
    int16_t rightWheelSpeed;

    int16_t commandVelocity;
    int16_t commandSteering;

    int16_t speedIntegralTorque;

    uint8_t robotMode;
    uint8_t statusFlags;
};

static_assert(
    sizeof(SlowFlightSample) == 24,
    "SlowFlightSample must be exactly 24 bytes");
