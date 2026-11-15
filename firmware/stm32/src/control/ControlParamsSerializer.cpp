#include "ControlParamsSerializer.h"

#include <cstdio>

std::size_t serializeControlParams(
    const ControlParams& p,
    char* buffer,
    std::size_t capacity)
{
    if (buffer == nullptr || capacity == 0)
        return 0;

    const int length = std::snprintf(
        buffer,
        capacity,
        "PARAMS "
        "kpBalance=%.6f "
        "kdBalance=%.6f "
        "kpSpeed=%.6f "
        "kiSpeed=%.6f "
        "maxSpeedIntegralTorque=%.6f "
        "kpYaw=%.6f "
        "kdYaw=%.6f "
        "wheelRadius=%.6f "
        "maxTorque=%.6f\n",
        static_cast<double>(p.kpBalance),
        static_cast<double>(p.kdBalance),
        static_cast<double>(p.kpSpeed),
        static_cast<double>(p.kiSpeed),
        static_cast<double>(p.maxSpeedIntegralTorque),
        static_cast<double>(p.kpYaw),
        static_cast<double>(p.kdYaw),
        static_cast<double>(p.wheelRadius),
        static_cast<double>(p.maxTorque));

    if (length < 0 ||
        static_cast<std::size_t>(length) >= capacity)
    {
        buffer[0] = '\0';
        return 0;
    }

    return static_cast<std::size_t>(length);
}
