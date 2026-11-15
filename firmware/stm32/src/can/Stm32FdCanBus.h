#pragma once

#include "CanBus.h"

#include <stm32g4xx.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_fdcan.h>


class Stm32FdCanBus : public CanBus
{
public:
    Stm32FdCanBus();

    bool begin();

    bool send(const CanFrame& frame) override;
    bool receive(CanFrame& frame) override;

private:
    FDCAN_HandleTypeDef handle_{};
};