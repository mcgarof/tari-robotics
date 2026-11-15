#include "BalanceController.h"
#include <algorithm>
#include <cmath>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

float normalizeAngle(float angle)
{
    while (angle > kPi)
        angle -= kTwoPi;

    while (angle < -kPi)
        angle += kTwoPi;

    return angle;
}
}

BalanceController::BalanceController(const ControlParams& params)
    : params_(params)
{
}

void BalanceController::reset()
{
    speedErrorFiltered_ = 0.0f;
    speedIntegral_ = 0.0f;
    yawReferenceInitialized_ = false;
    diagnostics_ = {};
}

MotorCommand BalanceController::update(
    const RobotState& state,
    const MotionCommand& command,
    float dt)
{
    const float balanceTorque =
        params_.kpBalance * state.pitch +
        params_.kdBalance * state.pitchRate;

    const float currentSpeed =
        0.5f *
        (state.leftWheelSpeed + state.rightWheelSpeed) *
        params_.wheelRadius;

    const float speedError =
        currentSpeed - command.velocity; // m/s

    speedErrorFiltered_ =
        0.7f * speedErrorFiltered_ +
        0.3f * speedError;

    speedIntegral_ += speedErrorFiltered_ * dt; // m

    // Anti-windup: limit the integral state so that its contribution
    // to the requested torque cannot exceed maxSpeedIntegralTorque.
    if (params_.kiSpeed != 0.0f)
    {
        const float integralLimit =
            params_.maxSpeedIntegralTorque / std::abs(params_.kiSpeed);

        speedIntegral_ = std::clamp(
            speedIntegral_,
            -integralLimit,
            integralLimit);
    }

    const float speedTorque =
        params_.kpSpeed * speedErrorFiltered_ +
        params_.kiSpeed * speedIntegral_;

    diagnostics_.speedIntegralTorque =
        params_.kiSpeed * speedIntegral_;

    if (!yawReferenceInitialized_)
    {
        yawReference_ = state.yaw;
        yawReferenceInitialized_ = true;
    }

    const float desiredYawRate = command.steering;

    // Integrate the requested yaw rate to obtain the desired heading.
    yawReference_ = normalizeAngle(
        yawReference_ + desiredYawRate * dt);

    const float yawError =
        normalizeAngle(state.yaw - yawReference_); // rad

    const float yawRateError =
        state.yawRate - desiredYawRate; // rad/s

    const float steeringTorque =
        params_.kpYaw * yawError +
        params_.kdYaw * yawRateError; // N·m

    MotorCommand output;

    output.leftTorque = std::clamp(
        balanceTorque + speedTorque - steeringTorque,
        -params_.maxTorque,
        params_.maxTorque);

    output.rightTorque = std::clamp(
        balanceTorque + speedTorque + steeringTorque,
        -params_.maxTorque,
        params_.maxTorque);

    return output;
}

void BalanceController::setParams(
    const ControlParams& params)
{
    params_ = params;
    reset();
}
