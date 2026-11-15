#pragma once

#include "RobotMode.h"
#include "SafetyStatus.h"

class SafetyManager
{
public:
    SafetyManager();

    RobotMode update(const SafetyStatus& status);

    RobotMode mode() const;

    void reset();

private:
    RobotMode mode_ = RobotMode::BOOT;
    bool enableReleaseRequired_ = false;
};