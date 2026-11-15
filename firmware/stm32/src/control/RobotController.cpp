#include "RobotController.h"
#include "ControlParamsValidator.h"

RobotController::RobotController(const ControlParams& controlParams)
    : balanceController_(controlParams)
{
}

ActuatorCommand RobotController::update(
    const RobotState& state,
    const MotionCommand& command,
    const SafetyStatus& safetyStatus,
    float dt)
{
    const RobotMode currentMode =
        safetyManager_.update(safetyStatus);

    if (currentMode == RobotMode::ACTIVE &&
        previousMode_ != RobotMode::ACTIVE)
    {
        balanceController_.reset();
    }

    ActuatorCommand output{
        .driveEnabled = false,
        .leftTorque = 0.0f,
        .rightTorque = 0.0f
    };

    if (currentMode == RobotMode::ACTIVE)
    {
        const MotorCommand motorCommand =
            balanceController_.update(
                state,
                command,
                dt);

        output.driveEnabled = true;
        output.leftTorque = motorCommand.leftTorque;
        output.rightTorque = motorCommand.rightTorque;
    }

    previousMode_ = currentMode;

    return output;
}

RobotMode RobotController::mode() const
{
    return safetyManager_.mode();
}

void RobotController::reset()
{
    safetyManager_.reset();
    balanceController_.reset();
    previousMode_ = RobotMode::BOOT;
}

const ControlDiagnostics& RobotController::diagnostics() const
{
    return balanceController_.diagnostics();
}

bool RobotController::setControlParams(
    const ControlParams& params)
{
    if (mode() != RobotMode::IDLE)
        return false;

    if (validateControlParams(params) !=
        ControlParamsValidationError::NONE)
    {
        return false;
    }

    balanceController_.setParams(params);
    return true;
}

const ControlParams& RobotController::controlParams() const
{
    return balanceController_.params();
}
