#pragma once

#include "../control/ActuatorCommand.h"

class MotorController
{
public:
    virtual ~MotorController() = default;

    virtual void apply(const ActuatorCommand& command) = 0;
};
