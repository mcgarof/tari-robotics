#include "Stm32FdCanBus.h"

namespace
{

uint32_t lengthToDlc(std::size_t length)
{
    static constexpr uint32_t dlc[] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };

    return dlc[length];
}

}

Stm32FdCanBus::Stm32FdCanBus()
{
    handle_.Instance = FDCAN1;
}

bool Stm32FdCanBus::begin()
{
    // Explicitly select HSE (24 MHz) as FDCAN clock source.
    RCC_PeriphCLKInitTypeDef peripheralClockConfig{};
    peripheralClockConfig.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    peripheralClockConfig.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;

    if (HAL_RCCEx_PeriphCLKConfig(&peripheralClockConfig) != HAL_OK)
    {
        return false;
    }

    // Enable peripheral and GPIO clocks.
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // PB8  -> FDCAN1_RX
    // PB9  -> FDCAN1_TX
    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;

    HAL_GPIO_Init(GPIOB, &gpio);

    // Classic CAN, 1 Mbit/s.
    //
    // FDCAN clock = 24 MHz
    // Prescaler   = 3
    // TQ clock    = 8 MHz
    //
    // 1 bit = Sync(1) + Seg1(5) + Seg2(2) = 8 TQ
    // 8 MHz / 8 = 1 Mbit/s
    handle_.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    handle_.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    handle_.Init.Mode = FDCAN_MODE_NORMAL;
    handle_.Init.AutoRetransmission = ENABLE;
    handle_.Init.TransmitPause = DISABLE;
    handle_.Init.ProtocolException = DISABLE;

    handle_.Init.NominalPrescaler = 3;
    handle_.Init.NominalSyncJumpWidth = 2;
    handle_.Init.NominalTimeSeg1 = 5;
    handle_.Init.NominalTimeSeg2 = 2;

    // Not used in Classic CAN, but initialize them explicitly.
    handle_.Init.DataPrescaler = 1;
    handle_.Init.DataSyncJumpWidth = 1;
    handle_.Init.DataTimeSeg1 = 1;
    handle_.Init.DataTimeSeg2 = 1;

    handle_.Init.StdFiltersNbr = 1;
    handle_.Init.ExtFiltersNbr = 0;
    handle_.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&handle_) != HAL_OK)
    {
        return false;
    }

    FDCAN_FilterTypeDef filter{};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;

    if (HAL_FDCAN_ConfigFilter(&handle_, &filter) != HAL_OK)
        return false;

    if (HAL_FDCAN_ConfigGlobalFilter(
            &handle_,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_Start(&handle_) != HAL_OK)
        return false;

    return true;
}

bool Stm32FdCanBus::send(const CanFrame& frame)
{
    if (frame.id > 0x7FF || frame.length > 8)
        return false;

    FDCAN_TxHeaderTypeDef header{};

    header.Identifier = frame.id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;

    header.DataLength = lengthToDlc(frame.length);

    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(
               &handle_,
               &header,
               const_cast<uint8_t*>(frame.data)) == HAL_OK;
}

bool Stm32FdCanBus::receive(CanFrame& frame)
{
    if (HAL_FDCAN_GetRxFifoFillLevel(
            &handle_,
            FDCAN_RX_FIFO0) == 0)
    {
        return false;
    }

    FDCAN_RxHeaderTypeDef header{};

    uint8_t data[8]{};

    if (HAL_FDCAN_GetRxMessage(
            &handle_,
            FDCAN_RX_FIFO0,
            &header,
            data) != HAL_OK)
    {
        return false;
    }

    if (header.IdType != FDCAN_STANDARD_ID ||
        header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return false;
    }

    frame.id = header.Identifier;

    switch (header.DataLength)
    {
    case FDCAN_DLC_BYTES_0: frame.length = 0; break;
    case FDCAN_DLC_BYTES_1: frame.length = 1; break;
    case FDCAN_DLC_BYTES_2: frame.length = 2; break;
    case FDCAN_DLC_BYTES_3: frame.length = 3; break;
    case FDCAN_DLC_BYTES_4: frame.length = 4; break;
    case FDCAN_DLC_BYTES_5: frame.length = 5; break;
    case FDCAN_DLC_BYTES_6: frame.length = 6; break;
    case FDCAN_DLC_BYTES_7: frame.length = 7; break;
    case FDCAN_DLC_BYTES_8: frame.length = 8; break;
    default:
        return false;
    }

    for (std::size_t i = 0; i < frame.length; ++i)
        frame.data[i] = data[i];

    return true;
}
