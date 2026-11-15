#pragma once

struct RobotState
{
    // Angles in radians, angular rates in rad/s.
    float pitch;            // rad
    float pitchRate;        // rad/s
    float yaw;              // rad
    float yawRate;          // rad/s
    // Wheel speeds in rad/s.
    float leftWheelSpeed;   // rad/s
    float rightWheelSpeed;  // rad/s
};

struct MotionCommand
{
    float velocity;  // m/s
    float steering;  // desired yaw rate, rad/s
};

struct MotorCommand
{
    float leftTorque;    // N·m
    float rightTorque;   // N·m
};

struct ControlDiagnostics
{
    float speedIntegralTorque;  // N·m
};

struct ControlParams
{
    float kpBalance;               // N·m/rad
    float kdBalance;               // N·m/(rad/s)

    float kpSpeed;                 // N·m/(m/s)
    float kiSpeed;                 // N·m/m
    float maxSpeedIntegralTorque;  // N·m

    float kpYaw;                   // N·m/rad
    float kdYaw;                   // N·m/(rad/s)

    float wheelRadius;             // m
    float maxTorque;               // N·m, maximum absolute torque per wheel
};

class BalanceController
{
public:
    explicit BalanceController(const ControlParams& params);

    MotorCommand update(
        const RobotState& state,
        const MotionCommand& command,
        float dt);

    void reset();

    const ControlDiagnostics& diagnostics() const
    {
        return diagnostics_;
    }

    void setParams(const ControlParams& params);

    const ControlParams& params() const
    {
        return params_;
    }

private:
    ControlParams params_;

    float speedErrorFiltered_ = 0.0f;
    float speedIntegral_ = 0.0f;
    float yawReference_ = 0.0f;
    bool yawReferenceInitialized_ = false;
    ControlDiagnostics diagnostics_{};
};
