#include "OdescMotorController.h"
#include "OdescCanProtocol.h"
#include "TorqueLimiter.h"
#include <cmath>

namespace
{
constexpr float kTwoPi =
    2.0f * 3.14159265358979323846f;
}

OdescMotorController::OdescMotorController(
    CanBus& canBus,
    uint8_t leftNodeId,
    uint8_t rightNodeId,
    uint32_t heartbeatTimeoutMs)
    : canBus_(canBus),
      leftNodeId_(leftNodeId),
      rightNodeId_(rightNodeId),
      heartbeatTimeoutMs_(heartbeatTimeoutMs)
{
}

void OdescMotorController::apply(
    const ActuatorCommand& command)
{
    lastApplyResult_ = {};

    if (!command.driveEnabled)
    {
        if (closedLoopRequestSent_)
        {
            const CanFrame leftZeroTorqueFrame =
                OdescCanProtocol::makeSetInputTorqueFrame(
                    leftNodeId_,
                    0.0f);

            const CanFrame rightZeroTorqueFrame =
                OdescCanProtocol::makeSetInputTorqueFrame(
                    rightNodeId_,
                    0.0f);

            canBus_.send(leftZeroTorqueFrame);
            canBus_.send(rightZeroTorqueFrame);

            const CanFrame leftFrame =
                OdescCanProtocol::makeSetAxisRequestedStateFrame(
                    leftNodeId_,
                    OdescCanProtocol::kAxisStateIdle);

            const CanFrame rightFrame =
                OdescCanProtocol::makeSetAxisRequestedStateFrame(
                    rightNodeId_,
                    OdescCanProtocol::kAxisStateIdle);

            canBus_.send(leftFrame);
            canBus_.send(rightFrame);

            closedLoopRequestSent_ = false;
        }

        return;
    }

    // Reject non-finite torque commands.
    if (!std::isfinite(command.leftTorque) ||
        !std::isfinite(command.rightTorque))
    {
        lastApplyResult_.invalidTorque = true;
        return;
    }

    const bool closedLoop =
        closedLoopActive(
            nowMs_,
            heartbeatTimeoutMs_);

    if (!closedLoop)
    {
        const bool closedLoopRequestDue =
            !closedLoopRequestSent_ ||
            static_cast<uint32_t>(
                nowMs_ - lastClosedLoopRequestMs_) >=
                kClosedLoopRetryMs;

        if (closedLoopRequestDue)
        {
            const CanFrame leftEnableFrame =
                OdescCanProtocol::makeSetAxisRequestedStateFrame(
                    leftNodeId_,
                    OdescCanProtocol::kAxisStateClosedLoopControl);

            const CanFrame rightEnableFrame =
                OdescCanProtocol::makeSetAxisRequestedStateFrame(
                    rightNodeId_,
                    OdescCanProtocol::kAxisStateClosedLoopControl);

            canBus_.send(leftEnableFrame);
            canBus_.send(rightEnableFrame);

            lastClosedLoopRequestMs_ = nowMs_;
            closedLoopRequestSent_ = true;
        }

        const CanFrame leftZeroTorqueFrame =
            OdescCanProtocol::makeSetInputTorqueFrame(
                leftNodeId_,
                0.0f);

        const CanFrame rightZeroTorqueFrame =
            OdescCanProtocol::makeSetInputTorqueFrame(
                rightNodeId_,
                0.0f);

        canBus_.send(leftZeroTorqueFrame);
        canBus_.send(rightZeroTorqueFrame);

        return;
    }

    // Apply the independent safety torque limit.
    const TorqueLimitResult limited = limitTorque(
        command.leftTorque,
        command.rightTorque,
        kSafetyMaxTorqueNm);

    lastApplyResult_.torqueClamped = limited.clamped;

    const CanFrame leftTorqueFrame =
        OdescCanProtocol::makeSetInputTorqueFrame(
            leftNodeId_,
            limited.leftTorque);

    const CanFrame rightTorqueFrame =
        OdescCanProtocol::makeSetInputTorqueFrame(
            rightNodeId_,
            limited.rightTorque);

    canBus_.send(leftTorqueFrame);
    canBus_.send(rightTorqueFrame);

}

void OdescMotorController::update(uint32_t nowMs)
{
    nowMs_ = nowMs;

    CanFrame frame{};

    while (canBus_.receive(frame))
    {
        OdescCanProtocol::Heartbeat heartbeat{};

        if (OdescCanProtocol::parseHeartbeat(
                frame,
                leftNodeId_,
                heartbeat))
        {
            status_.left.heartbeatReceived = true;
            status_.left.lastHeartbeatMs = nowMs;
            status_.left.axisError = heartbeat.axisError;
            status_.left.axisState = heartbeat.axisState;

            continue;
        }

        if (OdescCanProtocol::parseHeartbeat(
                frame,
                rightNodeId_,
                heartbeat))
        {
            status_.right.heartbeatReceived = true;
            status_.right.lastHeartbeatMs = nowMs;
            status_.right.axisError = heartbeat.axisError;
            status_.right.axisState = heartbeat.axisState;
        }

        OdescCanProtocol::EncoderEstimates estimates{};

        if (OdescCanProtocol::parseEncoderEstimates(
                frame,
                leftNodeId_,
                estimates))
        {
            leftWheelSpeedRadS_ =
                estimates.velocity * kTwoPi;

            leftEncoderLastUpdateMs_ = nowMs;
            leftEncoderReceived_ = true;

            continue;
        }

        if (OdescCanProtocol::parseEncoderEstimates(
                frame,
                rightNodeId_,
                estimates))
        {
            rightWheelSpeedRadS_ =
                estimates.velocity * kTwoPi;

            rightEncoderLastUpdateMs_ = nowMs;
            rightEncoderReceived_ = true;

            continue;
        }

        OdescCanProtocol::BusVoltageCurrent bus{};

        if (OdescCanProtocol::parseBusVoltageCurrent(
                frame,
                leftNodeId_,
                bus))
        {
            status_.power.received = true;
            status_.power.lastUpdateMs = nowMs;
            status_.power.busVoltage = bus.voltage;
            status_.power.busCurrent = bus.current;

            continue;
        }

        if (OdescCanProtocol::parseBusVoltageCurrent(
                frame,
                rightNodeId_,
                bus))
        {
            status_.power.received = true;
            status_.power.lastUpdateMs = nowMs;
            status_.power.busVoltage = bus.voltage;
            status_.power.busCurrent = bus.current;

            continue;
        }
    }
}

const OdescMotorStatus& OdescMotorController::status() const
{
    return status_;
}

float OdescMotorController::leftWheelSpeedRadS() const
{
    return leftWheelSpeedRadS_;
}

float OdescMotorController::rightWheelSpeedRadS() const
{
    return rightWheelSpeedRadS_;
}

bool OdescMotorController::wheelSpeedsFresh(
    uint32_t nowMs,
    uint32_t timeoutMs) const
{
    if (!leftEncoderReceived_ ||
        !rightEncoderReceived_)
    {
        return false;
    }

    const bool leftFresh =
        static_cast<uint32_t>(
            nowMs - leftEncoderLastUpdateMs_) <= timeoutMs;

    const bool rightFresh =
        static_cast<uint32_t>(
            nowMs - rightEncoderLastUpdateMs_) <= timeoutMs;

    return leftFresh && rightFresh;
}

bool OdescMotorController::closedLoopActive(
    uint32_t nowMs,
    uint32_t heartbeatTimeoutMs) const
{
    const bool leftHealthy =
        getAxisHealth(
            status_.left,
            nowMs,
            heartbeatTimeoutMs) == HealthState::OK;

    const bool rightHealthy =
        getAxisHealth(
            status_.right,
            nowMs,
            heartbeatTimeoutMs) == HealthState::OK;

    return
        leftHealthy &&
        rightHealthy &&
        status_.left.axisState ==
            OdescCanProtocol::kAxisStateClosedLoopControl &&
        status_.right.axisState ==
            OdescCanProtocol::kAxisStateClosedLoopControl;
}
