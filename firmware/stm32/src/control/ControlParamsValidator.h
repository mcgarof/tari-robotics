#pragma once

#include "BalanceController.h"

enum class ControlParamsValidationError
{
    NONE,
    NON_FINITE,
    OUT_OF_RANGE
};

ControlParamsValidationError validateControlParams(
    const ControlParams& params);
