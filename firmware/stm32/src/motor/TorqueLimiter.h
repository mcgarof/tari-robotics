
#pragma once

struct TorqueLimitResult
{
    float leftTorque;
    float rightTorque;
    bool clamped;
};

TorqueLimitResult limitTorque(
    float leftTorque,
    float rightTorque,
    float maxTorque);
