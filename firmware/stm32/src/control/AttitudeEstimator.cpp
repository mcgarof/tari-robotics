#include "AttitudeEstimator.h"
#include <cmath>

namespace
{
constexpr float kGravity = 9.80665f;

constexpr float kMinTrustedAccel =
    0.7f * kGravity;

constexpr float kMaxTrustedAccel =
    1.3f * kGravity;

constexpr float kMinTrustedAccelNormSquared =
    kMinTrustedAccel * kMinTrustedAccel;

constexpr float kMaxTrustedAccelNormSquared =
    kMaxTrustedAccel * kMaxTrustedAccel;
}

AttitudeEstimator::AttitudeEstimator(float timeConstant)
    : timeConstant_(timeConstant)
{
}

bool AttitudeEstimator::initialized() const
{
    return initialized_;
}

AttitudeState AttitudeEstimator::update(
    const ImuSample& sample,
    float dt)
{
    if (!initialized_)
    {
        const float accelNormSquared =
            sample.accelX * sample.accelX +
            sample.accelY * sample.accelY +
            sample.accelZ * sample.accelZ;

        if (accelNormSquared >= kMinTrustedAccelNormSquared &&
            accelNormSquared <= kMaxTrustedAccelNormSquared)
        {
            pitch_ = std::atan2(
                sample.accelX,
                sample.accelZ);

            initialized_ = true;
        }
    }
    else if (dt > 0.0f)
    {
        const float gyroPitch =
            pitch_ + sample.gyroY * dt;

        const float accelNormSquared =
            sample.accelX * sample.accelX +
            sample.accelY * sample.accelY +
            sample.accelZ * sample.accelZ;

        if (accelNormSquared >= kMinTrustedAccelNormSquared &&
            accelNormSquared <= kMaxTrustedAccelNormSquared)
        {
            const float accelPitch =
                std::atan2(sample.accelX, sample.accelZ);

            const float alpha =
                timeConstant_ / (timeConstant_ + dt);

            pitch_ =
                alpha * gyroPitch +
                (1.0f - alpha) * accelPitch;
        }
        else
        {
            // Accelerometer does not provide a meaningful gravity direction.
            // Temporarily rely on gyro integration only.
            pitch_ = gyroPitch;
        }

        yaw_ += sample.gyroZ * dt;
    }

    AttitudeState state{
        .pitch = pitch_,
        .pitchRate = sample.gyroY,
        .yaw = yaw_,
        .yawRate = sample.gyroZ
    };

    return state;
}

void AttitudeEstimator::reset()
{
    initialized_ = false;

    pitch_ = 0.0f;
    yaw_ = 0.0f;
}
