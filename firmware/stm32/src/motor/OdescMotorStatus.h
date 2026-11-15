#pragma once

#include <cstdint>
#include "../safety/HealthState.h"

struct OdescAxisStatus
{
    bool heartbeatReceived = false;
    uint32_t lastHeartbeatMs = 0;

    uint32_t axisError = 0;
    uint8_t axisState = 0;
};

struct OdescPowerStatus
{
    bool received = false;
    uint32_t lastUpdateMs = 0;

    float busVoltage = 0.0f;
    float busCurrent = 0.0f;
};

struct OdescMotorStatus
{
    OdescAxisStatus left;
    OdescAxisStatus right;

    OdescPowerStatus power;
};

inline bool isHeartbeatFresh(
    const OdescAxisStatus& status,
    uint32_t nowMs,
    uint32_t timeoutMs)
{
    if (!status.heartbeatReceived)
    {
        return false;
    }

    return (nowMs - status.lastHeartbeatMs) <= timeoutMs;
}

inline HealthState getAxisHealth(
    const OdescAxisStatus& status,
    uint32_t nowMs,
    uint32_t heartbeatTimeoutMs)
{
    if (!isHeartbeatFresh(
            status,
            nowMs,
            heartbeatTimeoutMs))
    {
        return HealthState::UNAVAILABLE;
    }

    if (status.axisError != 0)
    {
        return HealthState::FAULT;
    }

    return HealthState::OK;
}

inline HealthState getMotorHealth(
    const OdescMotorStatus& status,
    uint32_t nowMs,
    uint32_t heartbeatTimeoutMs)
{
    const HealthState leftHealth =
        getAxisHealth(
            status.left,
            nowMs,
            heartbeatTimeoutMs);

    const HealthState rightHealth =
        getAxisHealth(
            status.right,
            nowMs,
            heartbeatTimeoutMs);

    if (leftHealth == HealthState::FAULT ||
        rightHealth == HealthState::FAULT)
    {
        return HealthState::FAULT;
    }

    if (leftHealth == HealthState::UNAVAILABLE ||
        rightHealth == HealthState::UNAVAILABLE)
    {
        return HealthState::UNAVAILABLE;
    }

    return HealthState::OK;
}

inline bool isPowerStatusFresh(
    const OdescPowerStatus& status,
    uint32_t nowMs,
    uint32_t timeoutMs)
{
    if (!status.received)
    {
        return false;
    }

    return (nowMs - status.lastUpdateMs) <= timeoutMs;
}
