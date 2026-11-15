#pragma once

#include "MotorController.h"
#include "../can/CanBus.h"
#include "OdescMotorStatus.h"

struct ActuatorApplyResult
{
    bool torqueClamped = false;
    bool invalidTorque = false;
};

class OdescMotorController : public MotorController
{
public:
    OdescMotorController(
        CanBus& canBus,
        uint8_t leftNodeId,
        uint8_t rightNodeId,
        uint32_t heartbeatTimeoutMs);

    void apply(const ActuatorCommand& command) override;

    void update(uint32_t nowMs);

    const OdescMotorStatus& status() const;

    float leftWheelSpeedRadS() const;
    float rightWheelSpeedRadS() const;

    bool wheelSpeedsFresh(
        uint32_t nowMs,
        uint32_t timeoutMs) const;
    
    bool closedLoopActive(
        uint32_t nowMs,
        uint32_t heartbeatTimeoutMs) const;

    const ActuatorApplyResult& lastApplyResult() const
    {
        return lastApplyResult_;
    }

private:
    CanBus& canBus_;

    uint8_t leftNodeId_;
    uint8_t rightNodeId_;

    uint32_t nowMs_ = 0;
    uint32_t heartbeatTimeoutMs_;
    uint32_t lastClosedLoopRequestMs_ = 0;
    bool closedLoopRequestSent_ = false;

    static constexpr uint32_t kClosedLoopRetryMs = 100; // initial TBV
    static constexpr float kSafetyMaxTorqueNm = 5.0f; // initial TBV

    OdescMotorStatus status_{};

    float leftWheelSpeedRadS_ = 0.0f;
    float rightWheelSpeedRadS_ = 0.0f;

    uint32_t leftEncoderLastUpdateMs_ = 0;
    uint32_t rightEncoderLastUpdateMs_ = 0;

    bool leftEncoderReceived_ = false;
    bool rightEncoderReceived_ = false;
    
    ActuatorApplyResult lastApplyResult_{};
};
