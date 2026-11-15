#include "Lsm6dsv16xImu.h"

#include <Arduino.h>
#include <SPI.h>
#include <cmath>

namespace
{
constexpr float kMgToMetersPerSecondSquared =
    9.80665f / 1000.0f;

constexpr float kMdpsToRadiansPerSecond =
    3.14159265358979323846f / (180.0f * 1000.0f);
}

Lsm6dsv16xImu::Lsm6dsv16xImu(uint8_t chipSelectPin)
    : chipSelectPin_(chipSelectPin)
{
}

bool Lsm6dsv16xImu::begin()
{
    SPI.begin();

    pinMode(chipSelectPin_, OUTPUT);
    digitalWrite(chipSelectPin_, HIGH);

    if (!sensor_.begin(chipSelectPin_))
    {
        health_ = HealthState::UNAVAILABLE;
        return false;
    }

    if (!sensor_.deviceReset())
    {
        health_ = HealthState::FAULT;
        return false;
    }

    while (!sensor_.getDeviceReset())
    {
        delay(1);
    }

    if (!sensor_.enableBlockDataUpdate() ||
        !sensor_.setAccelDataRate(LSM6DSV16X_ODR_AT_960Hz) ||
        !sensor_.setAccelFullScale(LSM6DSV16X_4g) ||
        !sensor_.setGyroDataRate(LSM6DSV16X_ODR_AT_960Hz) ||
        !sensor_.setGyroFullScale(LSM6DSV16X_500dps))
    {
        health_ = HealthState::FAULT;
        return false;
    }

    health_ = HealthState::UNAVAILABLE;
    hasSample_ = false;
    lastSampleMs_ = 0;

    return true;
}

void Lsm6dsv16xImu::update()
{
    const uint32_t nowMs = millis();

    if (!sensor_.checkStatus())
    {
        if (!hasSample_ ||
            static_cast<uint32_t>(nowMs - lastSampleMs_) > kSampleTimeoutMs)
        {
            health_ = HealthState::UNAVAILABLE;
        }

        return;
    }

    sfe_lsm_data_t accelData{};
    sfe_lsm_data_t gyroData{};

    if (!sensor_.getAccel(&accelData) ||
        !sensor_.getGyro(&gyroData))
    {
        health_ = HealthState::FAULT;
        return;
    }

    ImuSample newSample{
        .accelX =
            accelData.xData * kMgToMetersPerSecondSquared,
        .accelY =
            accelData.yData * kMgToMetersPerSecondSquared,
        .accelZ =
            accelData.zData * kMgToMetersPerSecondSquared,
        .gyroX =
            gyroData.xData * kMdpsToRadiansPerSecond,
        .gyroY =
            gyroData.yData * kMdpsToRadiansPerSecond,
        .gyroZ =
            gyroData.zData * kMdpsToRadiansPerSecond
    };

    if (!std::isfinite(newSample.accelX) ||
        !std::isfinite(newSample.accelY) ||
        !std::isfinite(newSample.accelZ) ||
        !std::isfinite(newSample.gyroX) ||
        !std::isfinite(newSample.gyroY) ||
        !std::isfinite(newSample.gyroZ))
    {
        health_ = HealthState::FAULT;
        return;
    }

    sample_ = newSample;

    lastSampleMs_ = nowMs;
    hasSample_ = true;
    health_ = HealthState::OK;
}

ImuSample Lsm6dsv16xImu::sample() const
{
    return sample_;
}

HealthState Lsm6dsv16xImu::health() const
{
    return health_;
}
