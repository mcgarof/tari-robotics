#pragma once

#include <cstddef>
#include "BalanceController.h"

std::size_t serializeControlParams(
    const ControlParams& params,
    char* buffer,
    std::size_t capacity);
