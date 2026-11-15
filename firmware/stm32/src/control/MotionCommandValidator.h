#pragma once

#include "MotionCommandProtocol.h"

struct MotionCommandLimits
{
    float maxVelocity;      // m/s
    float maxSteeringRate;  // rad/s
};

class MotionCommandValidator
{
public:
    explicit MotionCommandValidator(const MotionCommandLimits& limits);

    bool valid(const MotionCommandProtocol::Packet& packet) const;

private:
    MotionCommandLimits limits_;
};
