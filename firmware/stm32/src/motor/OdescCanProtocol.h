#pragma once

#include <cstdint>
#include <cstring>

#include "../can/CanBus.h"

namespace OdescCanProtocol
{

static_assert(
    sizeof(float) == 4,
    "ODESC CAN protocol requires 32-bit float");

constexpr uint8_t kHeartbeat = 0x01;
constexpr uint8_t kSetAxisRequestedState = 0x07;
constexpr uint8_t kGetEncoderEstimates = 0x09;
constexpr uint8_t kSetControllerModes = 0x0B;
constexpr uint8_t kSetInputTorque = 0x0E;
constexpr uint8_t kGetIq = 0x14;
constexpr uint8_t kGetBusVoltageCurrent = 0x17;

constexpr uint32_t kAxisStateIdle = 1;
constexpr uint32_t kAxisStateClosedLoopControl = 8;

struct Heartbeat
{
    uint32_t axisError;
    uint8_t axisState;
};

struct EncoderEstimates
{
    float position;
    float velocity;
};

struct BusVoltageCurrent
{
    float voltage;
    float current;
};

inline void writeFloat32LE(
    uint8_t* destination,
    float value)
{
    uint32_t bits;

    std::memcpy(&bits, &value, sizeof(value));

    destination[0] = static_cast<uint8_t>(bits & 0xFF);
    destination[1] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    destination[2] = static_cast<uint8_t>((bits >> 16) & 0xFF);
    destination[3] = static_cast<uint8_t>((bits >> 24) & 0xFF);
}

inline void writeUint32LE(
    uint8_t* destination,
    uint32_t value)
{
    destination[0] = static_cast<uint8_t>(value & 0xFF);
    destination[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    destination[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    destination[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

inline uint32_t readUint32LE(
    const uint8_t* source)
{
    return
        static_cast<uint32_t>(source[0]) |
        (static_cast<uint32_t>(source[1]) << 8) |
        (static_cast<uint32_t>(source[2]) << 16) |
        (static_cast<uint32_t>(source[3]) << 24);
}

inline float readFloat32LE(
    const uint8_t* source)
{
    const uint32_t bits =
        readUint32LE(source);

    float value;
    std::memcpy(&value, &bits, sizeof(value));

    return value;
}

constexpr uint32_t makeCanId(
    uint8_t nodeId,
    uint8_t commandId)
{
    return
        (static_cast<uint32_t>(nodeId) << 5) |
        commandId;
}

inline CanFrame makeSetInputTorqueFrame(
    uint8_t nodeId,
    float torque)
{
    CanFrame frame{
        .id = makeCanId(nodeId, kSetInputTorque),
        .data = {},
        .length = 4
    };

    writeFloat32LE(
        frame.data,
        torque);

    return frame;
}

inline CanFrame makeSetAxisRequestedStateFrame(
    uint8_t nodeId,
    uint32_t requestedState)
{
    CanFrame frame{
        .id = makeCanId(
            nodeId,
            kSetAxisRequestedState),
        .data = {},
        .length = 4
    };

    writeUint32LE(
        frame.data,
        requestedState);

    return frame;
}

inline bool parseHeartbeat(
    const CanFrame& frame,
    uint8_t expectedNodeId,
    Heartbeat& heartbeat)
{
    if (frame.id != makeCanId(
            expectedNodeId,
            kHeartbeat))
    {
        return false;
    }

    if (frame.length < 5)
    {
        return false;
    }

    heartbeat.axisError =
        readUint32LE(frame.data);

    heartbeat.axisState =
        frame.data[4];

    return true;
}

inline bool parseEncoderEstimates(
    const CanFrame& frame,
    uint8_t expectedNodeId,
    EncoderEstimates& estimates)
{
    if (frame.id != makeCanId(
            expectedNodeId,
            kGetEncoderEstimates))
    {
        return false;
    }

    if (frame.length < 8)
    {
        return false;
    }

    estimates.position =
        readFloat32LE(frame.data);

    estimates.velocity =
        readFloat32LE(frame.data + 4);

    return true;
}

inline bool parseBusVoltageCurrent(
    const CanFrame& frame,
    uint8_t expectedNodeId,
    BusVoltageCurrent& bus)
{
    if (frame.id != makeCanId(
            expectedNodeId,
            kGetBusVoltageCurrent))
    {
        return false;
    }

    if (frame.length < 8)
    {
        return false;
    }

    bus.voltage =
        readFloat32LE(frame.data);

    bus.current =
        readFloat32LE(frame.data + 4);

    return true;
}

}
