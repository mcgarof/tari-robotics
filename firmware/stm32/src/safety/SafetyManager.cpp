#include "SafetyManager.h"

SafetyManager::SafetyManager() = default;

RobotMode SafetyManager::update(const SafetyStatus& status)
{
    switch (mode_)
    {
    case RobotMode::BOOT:
        if (status.imu == HealthState::OK &&
            status.motorController == HealthState::OK &&
            status.wheelFeedback == HealthState::OK &&
            status.control == HealthState::OK)
        {
            mode_ = RobotMode::IDLE;

            if (status.enableRequested)
            {
                enableReleaseRequired_ = true;
            }
        }
        break;

    case RobotMode::IDLE:
        if (enableReleaseRequired_)
        {
            if (!status.enableRequested)
            {
                enableReleaseRequired_ = false;
            }
        }
        else if (status.enableRequested &&
            status.imu == HealthState::OK &&
            status.motorController == HealthState::OK &&
            status.wheelFeedback == HealthState::OK &&
            status.control == HealthState::OK &&
            !status.tiltExceeded)
        {
            mode_ = RobotMode::ACTIVE;
        }
        break;

    case RobotMode::ACTIVE:
        if (status.imu == HealthState::FAULT ||
            status.motorController == HealthState::FAULT ||
            status.wheelFeedback == HealthState::FAULT ||
            status.control == HealthState::FAULT)
        {
            mode_ = RobotMode::FAULT;
        }
        else if (status.imu == HealthState::UNAVAILABLE ||
                status.motorController == HealthState::UNAVAILABLE ||
                status.wheelFeedback == HealthState::UNAVAILABLE ||
                status.control == HealthState::UNAVAILABLE ||
                status.tiltExceeded)
        {
            mode_ = RobotMode::SAFE_STOP;
        }
        else if (!status.enableRequested)
        {
            mode_ = RobotMode::IDLE;
        }
        break;

    case RobotMode::SAFE_STOP:
    case RobotMode::FAULT:
        if (status.resetRequested &&
            status.imu == HealthState::OK &&
            status.motorController == HealthState::OK &&
            status.wheelFeedback == HealthState::OK &&
            status.control == HealthState::OK &&
            !status.tiltExceeded)
        {
            mode_ = RobotMode::IDLE;
            enableReleaseRequired_ = true;
        }
        break;
    }

    return mode_;
}

RobotMode SafetyManager::mode() const
{
    return mode_;
}

void SafetyManager::reset()
{
    mode_ = RobotMode::BOOT;
    enableReleaseRequired_ = false;
}