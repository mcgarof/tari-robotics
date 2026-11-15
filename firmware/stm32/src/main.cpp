#include <Arduino.h>

#include "can/Stm32FdCanBus.h"
#include "motor/OdescMotorController.h"
#include "control/RobotController.h"
#include "safety/SafetyStatus.h"
#include "control/ControlWatchdog.h"
#include "control/CommandWatchdog.h"
#include "control/AttitudeEstimator.h"
#include "imu/Lsm6dsv16xImu.h"
#include "control/MotionCommandReceiver.h"
#include "safety/TiltMonitor.h"
#include "safety/SafetyInputs.h"
#include "observability/Event.h"
#include "observability/EventRecorder.h"
#include <cstdio>
#include "observability/FlightRecorder.h"
#include "observability/FastFlightSample.h"
#include "observability/SlowFlightSample.h"
#include "observability/SampleDecimator.h"
#include "observability/ObservabilityDump.h"
#include "observability/FlightDumpPublisher.h"
#include "observability/FlightRecorderSerializer.h"
#include "observability/DebugCommandParser.h"
#include "observability/EventDumpPublisher.h"
#include "control/ControlParamsSerializer.h"
#include "observability/NonBlockingWriter.h"

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kMotorHeartbeatTimeoutMs = 200;
constexpr uint8_t kLeftMotorNodeId = 0;
constexpr uint8_t kRightMotorNodeId = 1;
constexpr float kWheelRadiusM = 0.08255f; // initial TBV

const ControlParams kControlParams{
    .kpBalance = 0.0f,
    .kdBalance = 0.0f,
    .kpSpeed = 0.0f,
    .kiSpeed = 0.0f,
    .maxSpeedIntegralTorque = 0.0f,
    .kpYaw = 0.0f,
    .kdYaw = 0.0f,
    .wheelRadius = kWheelRadiusM,
    .maxTorque = 5.0f // initial TBV
};

constexpr uint32_t kControlPeriodUs = 1000;
constexpr uint32_t kControlOverrunUs = 2000;
constexpr uint32_t kControlLostUs = 10000;
constexpr uint32_t kMaxConsecutiveControlOverruns = 3;

constexpr uint8_t kImuChipSelectPin = PB6;

constexpr uint32_t kWheelFeedbackTimeoutMs = 30;

constexpr uint32_t kCommandTimeoutMs = 200;

constexpr uint32_t kCommandBaudRate = 115200;
constexpr uint32_t kDebugBaudRate = 115200;

constexpr float kMaxCommandVelocity = 2.0f;       // m/s, valore iniziale TBV
constexpr float kMaxCommandSteeringRate = 1.5f;  // rad/s, valore iniziale TBV

constexpr float degreesToRadians(float degrees)
{
    return degrees * kPi / 180.0f;
}

constexpr float kTiltTripThresholdRad = degreesToRadians(40.0f); // TBV
constexpr float kTiltRecoveryThresholdRad = degreesToRadians(30.0f); // TBV

constexpr uint8_t kEnablePin = PB4;
constexpr uint8_t kSafetyResetPin = PB5;

constexpr std::size_t kEventRecorderCapacity = 64;

constexpr uint8_t kSlowFlightDecimation = 5;

constexpr std::size_t kParamsResponseBufferSize = 256;

char paramsResponseBuffer[kParamsResponseBufferSize]{};
NonBlockingWriter paramsResponseWriter;

SampleDecimator slowFlightDecimator{kSlowFlightDecimation};

EventRecorder<kEventRecorderCapacity> eventRecorder;

EventDumpPublisher<kEventRecorderCapacity>eventDumpPublisher{serializeEvent};

FlightRecorder<FastFlightSample, kFastFlightRecorderCapacity> fastFlightRecorder;
FlightRecorder<SlowFlightSample, kSlowFlightRecorderCapacity> slowFlightRecorder;

ObservabilityDump observabilityDump;

FlightDumpPublisher<
    FastFlightSample,
    kFastFlightRecorderCapacity>
    fastFlightDumpPublisher{
        serializeFastFlightSample};

FlightDumpPublisher<
    SlowFlightSample,
    kSlowFlightRecorderCapacity>
    slowFlightDumpPublisher{
        serializeSlowFlightSample};

DebugCommandParser debugCommandParser;

SafetyInputs safetyInputs;

Stm32FdCanBus canBus;

OdescMotorController motorController(
    canBus,
    kLeftMotorNodeId,
    kRightMotorNodeId,
    kMotorHeartbeatTimeoutMs);

RobotController robotController(kControlParams);

uint32_t lastControlUs = 0;

ControlWatchdog controlWatchdog(
    kControlOverrunUs,
    kControlLostUs,
    kMaxConsecutiveControlOverruns);

CommandWatchdog commandWatchdog(kCommandTimeoutMs);

MotionCommand receivedCommand{
    .velocity = 0.0f,
    .steering = 0.0f
};

Lsm6dsv16xImu imu(kImuChipSelectPin);
AttitudeEstimator attitudeEstimator(0.5f);
AttitudeState attitude{};

RobotMode previousRobotMode = RobotMode::BOOT;

bool invalidTorqueFault = false;
bool previousPhysicalTiltExceeded = false;
bool previousCommandTimedOut = false;
HealthState previousImuHealth = HealthState::UNAVAILABLE;
HealthState previousMotorHealth = HealthState::UNAVAILABLE;
bool motorWasAvailable = false;

Uart commandSerial(PB_11, PB_10); // RX, TX

MotionCommandReceiver commandReceiver({
    .maxVelocity = kMaxCommandVelocity,
    .maxSteeringRate = kMaxCommandSteeringRate
});

TiltMonitor tiltMonitor(
    kTiltTripThresholdRad,
    kTiltRecoveryThresholdRad);

void processCommandSerial()
{
    while (commandSerial.available() > 0) {
        const int value = commandSerial.read();
        if (value < 0) {
            break;
        }

        MotionCommandProtocol::Packet packet{};

        if (commandReceiver.pushByte(
                static_cast<uint8_t>(value), packet)) {

            receivedCommand.velocity = packet.velocity;
            receivedCommand.steering = packet.steering;

            // Timestamp preso quando il comando valido è stato
            // effettivamente ricevuto e accettato.
            commandWatchdog.notifyCommandReceived(millis());
        }
    }
}

void updateObservabilityDump()
{
    switch (observabilityDump.state())
    {
    case ObservabilityDumpState::IDLE:
        return;

    case ObservabilityDumpState::FAST:
        fastFlightDumpPublisher.update(
            fastFlightRecorder,
            Serial);

        if (!fastFlightDumpPublisher.active())
            observabilityDump.stop();

        return;

    case ObservabilityDumpState::SLOW:
        slowFlightDumpPublisher.update(
            slowFlightRecorder,
            Serial);

        if (!slowFlightDumpPublisher.active())
            observabilityDump.stop();

        return;

    case ObservabilityDumpState::EVENTS:
        eventDumpPublisher.update(
            eventRecorder,
            Serial);

        if (!eventDumpPublisher.active())
            observabilityDump.stop();

        return;
    }
}

bool startObservabilityDump(ObservabilityDumpState state)
{
    if (paramsResponseWriter.busy())
        return false;

    if (observabilityDump.active())
        return false;

    switch (state)
    {
    case ObservabilityDumpState::FAST:
        if (!fastFlightRecorder.frozen())
            return false;

        if (!fastFlightDumpPublisher.start())
            return false;
        break;

    case ObservabilityDumpState::SLOW:
        if (!slowFlightRecorder.frozen())
            return false;

        if (!slowFlightDumpPublisher.start())
            return false;
        break;

    case ObservabilityDumpState::EVENTS:
        if (!eventDumpPublisher.start())
            return false;
        break;

    case ObservabilityDumpState::IDLE:
        return false;
    }

    if (!observabilityDump.start(state))
    {
        // Defensive rollback. This should not normally happen.
        if (state == ObservabilityDumpState::FAST)
            fastFlightDumpPublisher.reset();
        else if (state == ObservabilityDumpState::SLOW)
            slowFlightDumpPublisher.reset();
        else if (state == ObservabilityDumpState::EVENTS)
            eventDumpPublisher.reset();

        return false;
    }

    return true;
}

void updateParamsResponse()
{
    if (paramsResponseWriter.busy())
        paramsResponseWriter.update(Serial);

    if (paramsResponseWriter.complete())
        paramsResponseWriter.reset();
}

void processDebugSerial()
{
    while (Serial.available() > 0)
    {
        const int value = Serial.read();

        if (value < 0)
            break;

        const DebugCommand command =
            debugCommandParser.push(
                static_cast<char>(value));

        switch (command)
        {
        case DebugCommand::DUMP_EVENTS:
            startObservabilityDump(
                ObservabilityDumpState::EVENTS);
            break;

        case DebugCommand::DUMP_FAST:
            startObservabilityDump(
                ObservabilityDumpState::FAST);
            break;

        case DebugCommand::DUMP_SLOW:
            startObservabilityDump(
                ObservabilityDumpState::SLOW);
            break;

        case DebugCommand::CLEAR_FLIGHT:
            if (!observabilityDump.active())
            {
                fastFlightRecorder.clear();
                slowFlightRecorder.clear();
            }
            break;

        case DebugCommand::REARM_FLIGHT:
            if (!observabilityDump.active())
            {
                fastFlightRecorder.rearm();
                slowFlightRecorder.rearm();
            }
            break;

        case DebugCommand::GET_PARAMS:
        {
            // if (robotController.mode() != RobotMode::IDLE)
            //     break;

            const std::size_t length =
                serializeControlParams(
                    robotController.controlParams(),
                    paramsResponseBuffer,
                    sizeof(paramsResponseBuffer));

            if (length > 0)
                paramsResponseWriter.begin(
                    paramsResponseBuffer, length);

            break;
        }

        case DebugCommand::NONE:
            break;
        }
    }
}

}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(kDebugBaudRate);

    if (!canBus.begin())
    {
        Serial.println("ERROR CANBUS_INIT_FAILED");

        while (true)
        {
            processDebugSerial();
            updateParamsResponse();
            delay(1);
        }
    }

    if (!imu.begin())
    {
        Serial.println("ERROR IMU_INIT_FAILED");

        while (true)
        {
            processDebugSerial();
            updateParamsResponse();
            delay(1);
        }
    }

    commandSerial.begin(kCommandBaudRate);

    pinMode(kEnablePin, INPUT_PULLDOWN);
    pinMode(kSafetyResetPin, INPUT_PULLDOWN);
}

void loop()
{
    processCommandSerial();

    const uint32_t nowMs = millis();

    // Housekeeping CAN: non aspetta il control cycle.
    motorController.update(nowMs);

    imu.update();

    const HealthState imuHealth = imu.health();

    if (imuHealth == HealthState::FAULT &&
        previousImuHealth != HealthState::FAULT)
    {
        eventRecorder.record(
            micros(),
            EventId::IMU_FAULT,
            EventSeverity::ERROR
        );
    }

    previousImuHealth = imuHealth;

    const uint32_t nowUs = micros();
    const uint32_t elapsedUs = nowUs - lastControlUs;

    if (elapsedUs < kControlPeriodUs)
    {
        processDebugSerial();
        updateObservabilityDump();
        updateParamsResponse();
        return;
    }

    const HealthState controlHealth =
        controlWatchdog.update(elapsedUs);

    if (elapsedUs > kControlOverrunUs)
    {
        eventRecorder.record(
            nowUs,
            EventId::CONTROL_OVERRUN,
            EventSeverity::WARNING
        );

        // A missed control interval cannot be recovered.
        // Resync instead of running catch-up cycles.
        lastControlUs = nowUs;
    }
    else
    {
        // Preserve the nominal schedule under normal timing jitter.
        lastControlUs += kControlPeriodUs;
    }

    const float dt =
        static_cast<float>(elapsedUs) * 1.0e-6f;

    if (imuHealth == HealthState::OK &&
        controlHealth == HealthState::OK)
    {
        attitude =
            attitudeEstimator.update(imu.sample(), dt);
    }

    const HealthState motorHealth =
        getMotorHealth(
            motorController.status(),
            nowMs,
            kMotorHeartbeatTimeoutMs);

    if (motorHealth == HealthState::OK)
    {
        motorWasAvailable = true;
    }

    if (motorWasAvailable &&
        motorHealth == HealthState::UNAVAILABLE &&
        previousMotorHealth != HealthState::UNAVAILABLE)
    {
        eventRecorder.record(
            micros(),
            EventId::MOTOR_HEARTBEAT_LOST,
            EventSeverity::ERROR
        );
    }

    if (motorHealth == HealthState::FAULT &&
        previousMotorHealth != HealthState::FAULT)
    {
        eventRecorder.record(
            micros(),
            EventId::MOTOR_FAULT,
            EventSeverity::ERROR
        );
    }

    previousMotorHealth = motorHealth;

    const bool physicalTiltExceeded =
        tiltMonitor.update(attitude.pitch);

    const bool tiltExceeded =
        !attitudeEstimator.initialized() ||
        physicalTiltExceeded;

    if (physicalTiltExceeded && !previousPhysicalTiltExceeded)
    {
        eventRecorder.record(
            micros(),
            EventId::TILT_EXCEEDED,
            EventSeverity::ERROR
        );
    }

    previousPhysicalTiltExceeded = physicalTiltExceeded;

    safetyInputs.update(
        digitalRead(kEnablePin) == HIGH,
        digitalRead(kSafetyResetPin) == HIGH,
        millis());

    const bool resetInvalidTorqueFault =
        invalidTorqueFault &&
        safetyInputs.resetRequested() &&
        !safetyInputs.enableRequested();

    SafetyStatus safetyStatus{
        .imu = imuHealth,
        .motorController = motorHealth,
        .wheelFeedback =
            motorController.wheelSpeedsFresh(nowMs, kWheelFeedbackTimeoutMs)
                ? HealthState::OK
                : HealthState::UNAVAILABLE,
        .control = (invalidTorqueFault && !resetInvalidTorqueFault)
            ? HealthState::FAULT
            : controlHealth,
        .tiltExceeded = tiltExceeded,
        .enableRequested = safetyInputs.enableRequested(),
        .resetRequested = safetyInputs.resetRequested()
    };

    const RobotState robotState{
        .pitch = attitude.pitch,
        .pitchRate = attitude.pitchRate,
        .yaw = attitude.yaw,
        .yawRate = attitude.yawRate,
        .leftWheelSpeed = motorController.leftWheelSpeedRadS(),
        .rightWheelSpeed = motorController.rightWheelSpeedRadS()
    };

    const bool commandFresh =
        commandWatchdog.commandFresh(nowMs);

    const bool commandTimedOut =
        commandWatchdog.commandReceived() && !commandFresh;

    if (commandTimedOut && !previousCommandTimedOut)
    {
        eventRecorder.record(
            micros(),
            EventId::COMMAND_TIMEOUT,
            EventSeverity::WARNING
        );
    }

    previousCommandTimedOut = commandTimedOut;

    const MotionCommand effectiveCommand =
        commandFresh
            ? receivedCommand
            : MotionCommand{
                .velocity = 0.0f,
                .steering = 0.0f
            };

    const ActuatorCommand actuatorCommand =
        robotController.update(
            robotState,
            effectiveCommand,
            safetyStatus,
            dt);

    FastFlightSample fastSample{};

    fastSample.pitch =
        encodeFlightValue(
            robotState.pitch,
            kAngleScale);

    fastSample.pitchRate =
        encodeFlightValue(
            robotState.pitchRate,
            kAngularRateScale);

    fastSample.yaw =
        encodeFlightValue(
            robotState.yaw,
            kAngleScale);

    fastSample.yawRate =
        encodeFlightValue(
            robotState.yawRate,
            kAngularRateScale);

    fastSample.leftWheelSpeed =
        encodeFlightValue(
            robotState.leftWheelSpeed,
            kWheelSpeedScale);

    fastSample.rightWheelSpeed =
        encodeFlightValue(
            robotState.rightWheelSpeed,
            kWheelSpeedScale);

    fastSample.leftTorque =
        encodeFlightValue(
            actuatorCommand.leftTorque,
            kTorqueScale);

    fastSample.rightTorque =
        encodeFlightValue(
            actuatorCommand.rightTorque,
            kTorqueScale);

    fastFlightRecorder.record(
        nowUs,
        fastSample);

    const RobotMode currentRobotMode =
        robotController.mode();

    if (resetInvalidTorqueFault &&
        currentRobotMode == RobotMode::IDLE &&
        safetyStatus.control == HealthState::OK)
    {
        invalidTorqueFault = false;
    }

    if (slowFlightDecimator.tick())
    {
        SlowFlightSample slowSample{};

        const ControlDiagnostics& controlDiagnostics =
            robotController.diagnostics();

        slowSample.pitch =
            encodeFlightValue(
                robotState.pitch,
                kAngleScale);

        slowSample.pitchRate =
            encodeFlightValue(
                robotState.pitchRate,
                kAngularRateScale);

        slowSample.yaw =
            encodeFlightValue(
                robotState.yaw,
                kAngleScale);

        slowSample.yawRate =
            encodeFlightValue(
                robotState.yawRate,
                kAngularRateScale);

        slowSample.leftWheelSpeed =
            encodeFlightValue(
                robotState.leftWheelSpeed,
                kWheelSpeedScale);

        slowSample.rightWheelSpeed =
            encodeFlightValue(
                robotState.rightWheelSpeed,
                kWheelSpeedScale);

        slowSample.commandVelocity =
            encodeFlightValue(
                effectiveCommand.velocity,
                kVelocityScale);

        slowSample.commandSteering =
            encodeFlightValue(
                effectiveCommand.steering,
                kAngularRateScale);

        slowSample.speedIntegralTorque =
            encodeFlightValue(
                controlDiagnostics.speedIntegralTorque,
                kTorqueScale);

        slowSample.robotMode =
            static_cast<uint8_t>(currentRobotMode);

        uint8_t statusFlags = 0;

        if (imuHealth == HealthState::OK)
            statusFlags |= SLOW_IMU_OK;

        if (motorHealth == HealthState::OK)
            statusFlags |= SLOW_MOTOR_OK;

        if (safetyStatus.wheelFeedback == HealthState::OK)
            statusFlags |= SLOW_WHEEL_FEEDBACK_OK;

        if (controlHealth == HealthState::OK)
            statusFlags |= SLOW_CONTROL_OK;

        if (commandFresh)
            statusFlags |= SLOW_COMMAND_FRESH;

        if (tiltExceeded)
            statusFlags |= SLOW_TILT_EXCEEDED;

        if (safetyInputs.enableRequested())
            statusFlags |= SLOW_ENABLE_REQUESTED;

        if (safetyInputs.resetRequested())
            statusFlags |= SLOW_RESET_REQUESTED;

        slowSample.statusFlags = statusFlags;

        slowFlightRecorder.record(
            nowUs,
            slowSample);
    }

    if (currentRobotMode != previousRobotMode)
    {
        if (currentRobotMode == RobotMode::SAFE_STOP)
        {
            eventRecorder.record(
                micros(),
                EventId::SAFE_STOP_ENTERED,
                EventSeverity::ERROR
            );

            fastFlightRecorder.freeze();
            slowFlightRecorder.freeze();
        }
        else if (currentRobotMode == RobotMode::FAULT)
        {
            eventRecorder.record(
                micros(),
                EventId::FAULT_ENTERED,
                EventSeverity::CRITICAL
            );

            fastFlightRecorder.freeze();
            slowFlightRecorder.freeze();
        }
    }

    if ((previousRobotMode == RobotMode::SAFE_STOP ||
        previousRobotMode == RobotMode::FAULT) &&
        currentRobotMode == RobotMode::IDLE)
    {
        attitudeEstimator.reset();
        attitude = {};
    }

    previousRobotMode = currentRobotMode;

    motorController.apply(actuatorCommand);

    if (motorController.lastApplyResult().invalidTorque)
    {
        invalidTorqueFault = true;
    }
}
