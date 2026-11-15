#include "MotionCommandValidator.h"

#include <cmath>

MotionCommandValidator::MotionCommandValidator(
    const MotionCommandLimits& limits)
    : limits_(limits)
{
}

bool MotionCommandValidator::valid(
    const MotionCommandProtocol::Packet& packet) const
{
    if (!std::isfinite(packet.velocity) ||
        !std::isfinite(packet.steering))
    {
        return false;
    }

    if (std::abs(packet.velocity) > limits_.maxVelocity ||
        std::abs(packet.steering) > limits_.maxSteeringRate)
    {
        return false;
    }

    return true;
}
