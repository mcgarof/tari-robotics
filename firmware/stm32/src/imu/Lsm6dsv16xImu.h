#pragma once

#include "Imu.h"

#include <SparkFun_LSM6DSV16X.h>

class Lsm6dsv16xImu : public Imu
{
public:
    explicit Lsm6dsv16xImu(uint8_t chipSelectPin);

    bool begin();

    void update() override;

    ImuSample sample() const override;

    HealthState health() const override;

private:
    static constexpr uint32_t kSampleTimeoutMs = 10;

    uint8_t chipSelectPin_;

    SparkFun_LSM6DSV16X_SPI sensor_;

    ImuSample sample_{};

    HealthState health_ = HealthState::UNAVAILABLE;

    uint32_t lastSampleMs_ = 0;
    bool hasSample_ = false;
};
