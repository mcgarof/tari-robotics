#pragma once

#include "../imu/Imu.h"

struct AttitudeState
{
    // Angles in radians, angular rates in rad/s.
    float pitch;
    float pitchRate;

    float yaw;
    float yawRate;
};

class AttitudeEstimator
{
public:
    explicit AttitudeEstimator(float timeConstant = 0.5f);

    AttitudeState update(
        const ImuSample& sample,
        float dt);

    void reset();

    bool initialized() const;

private:
    // Complementary filter time constant in seconds.
    float timeConstant_;

    bool initialized_ = false;

    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
};
