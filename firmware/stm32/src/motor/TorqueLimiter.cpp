
#include "TorqueLimiter.h"

#include <algorithm>
#include <cmath>

TorqueLimitResult limitTorque(
    float leftTorque,
    float rightTorque,
    float maxTorque)
{
    const float left =
        std::clamp(leftTorque, -maxTorque, maxTorque);

    const float right =
        std::clamp(rightTorque, -maxTorque, maxTorque);

    return {
        left,
        right,
        left != leftTorque || right != rightTorque
    };
}
