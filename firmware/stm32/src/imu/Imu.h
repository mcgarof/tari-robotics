#pragma once

#include "../safety/HealthState.h"

struct ImuSample
{
    // Linear acceleration in m/s^2.
    float accelX;
    float accelY;
    float accelZ;

    // Angular velocity in rad/s.
    float gyroX;
    float gyroY;
    float gyroZ;
};

class Imu
{
public:
    virtual ~Imu() = default;

    virtual void update() = 0;

    virtual ImuSample sample() const = 0;

    virtual HealthState health() const = 0;
};
