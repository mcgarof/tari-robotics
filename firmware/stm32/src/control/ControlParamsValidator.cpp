
#include "ControlParamsValidator.h"

#include <cmath>

ControlParamsValidationError validateControlParams(
    const ControlParams& params)
{
    if (!std::isfinite(params.kpBalance) ||
        !std::isfinite(params.kdBalance) ||
        !std::isfinite(params.kpSpeed) ||
        !std::isfinite(params.kiSpeed) ||
        !std::isfinite(params.maxSpeedIntegralTorque) ||
        !std::isfinite(params.kpYaw) ||
        !std::isfinite(params.kdYaw) ||
        !std::isfinite(params.wheelRadius) ||
        !std::isfinite(params.maxTorque))
    {
        return ControlParamsValidationError::NON_FINITE;
    }

    if (params.wheelRadius <= 0.0f ||
        params.maxTorque <= 0.0f ||
        params.maxSpeedIntegralTorque < 0.0f)
    {
        return ControlParamsValidationError::OUT_OF_RANGE;
    }

    return ControlParamsValidationError::NONE;
}
