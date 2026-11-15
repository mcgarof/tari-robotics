#pragma once

#include <cstdint>
#include <cmath>
#include <limits>

constexpr float kAngleScale = 1000.0f;       // rad -> mrad
constexpr float kAngularRateScale = 1000.0f; // rad/s -> mrad/s
constexpr float kWheelSpeedScale = 100.0f;   // rad/s -> 0.01 rad/s
constexpr float kTorqueScale = 1000.0f;      // N·m -> mN·m
constexpr float kVelocityScale = 1000.0f;    // m/s -> mm/s

constexpr int16_t kInvalidFlightValue =
    std::numeric_limits<int16_t>::min();

constexpr int16_t kMinFlightValue =
    std::numeric_limits<int16_t>::min() + 1;

constexpr int16_t kMaxFlightValue =
    std::numeric_limits<int16_t>::max();

inline int16_t encodeFlightValue(
    float value,
    float scale)
{
    if (!std::isfinite(value))
        return kInvalidFlightValue;

    const float scaled = value * scale;

    if (scaled >= static_cast<float>(kMaxFlightValue))
        return kMaxFlightValue;

    if (scaled <= static_cast<float>(kMinFlightValue))
        return kMinFlightValue;

    return static_cast<int16_t>(scaled);
}
