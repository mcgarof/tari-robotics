#pragma once

#include "HealthState.h"

struct SafetyStatus
{
    HealthState imu;
    HealthState motorController;
    HealthState wheelFeedback;
    HealthState control;

    bool tiltExceeded;

    bool enableRequested;
    bool resetRequested;
};
