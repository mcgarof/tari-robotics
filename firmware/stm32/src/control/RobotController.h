#pragma once

#include "BalanceController.h"
#include "../safety/SafetyManager.h"
#include "ActuatorCommand.h"

class RobotController
{
public:
    explicit RobotController(const ControlParams& controlParams);

    ActuatorCommand update(
        const RobotState& state,
        const MotionCommand& command,
        const SafetyStatus& safetyStatus,
        float dt);

    RobotMode mode() const;

    void reset();

    const ControlDiagnostics& diagnostics() const;

    bool setControlParams(const ControlParams& params);
    const ControlParams& controlParams() const;

private:
    SafetyManager safetyManager_;
    BalanceController balanceController_;

    RobotMode previousMode_ = RobotMode::BOOT;
};