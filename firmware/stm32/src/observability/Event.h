#pragma once

#include <cstdint>

enum class EventId : uint16_t {
    CONTROL_OVERRUN,
    IMU_FAULT,
    TILT_EXCEEDED,
    COMMAND_TIMEOUT,
    MOTOR_HEARTBEAT_LOST,
    MOTOR_FAULT,
    SAFE_STOP_ENTERED,
    FAULT_ENTERED,
    PARAM_CHANGE
};

enum class EventSeverity : uint8_t {
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

struct Event {
    uint32_t timestampUs;
    uint32_t sequence;
    EventId id;
    EventSeverity severity;
};
