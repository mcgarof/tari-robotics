#include "MotionCommandProtocol.h"

#include <cstring>

namespace MotionCommandProtocol
{

namespace
{

constexpr uint16_t kCrcPolynomial = 0x1021;
constexpr uint16_t kCrcInitial = 0xFFFF;

uint16_t crc16CcittFalse(
    const uint8_t* data,
    std::size_t size)
{
    uint16_t crc = kCrcInitial;

    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (int bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x8000) != 0)
            {
                crc =
                    static_cast<uint16_t>(
                        (crc << 1) ^ kCrcPolynomial);
            }
            else
            {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

void writeUint16LE(uint8_t* destination, uint16_t value)
{
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8);
}

void writeUint32LE(uint8_t* destination, uint32_t value)
{
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8);
    destination[2] = static_cast<uint8_t>(value >> 16);
    destination[3] = static_cast<uint8_t>(value >> 24);
}

uint16_t readUint16LE(const uint8_t* source)
{
    return
        static_cast<uint16_t>(source[0]) |
        (static_cast<uint16_t>(source[1]) << 8);
}

uint32_t readUint32LE(const uint8_t* source)
{
    return
        static_cast<uint32_t>(source[0]) |
        (static_cast<uint32_t>(source[1]) << 8) |
        (static_cast<uint32_t>(source[2]) << 16) |
        (static_cast<uint32_t>(source[3]) << 24);
}

void writeFloat32LE(uint8_t* destination, float value)
{
    static_assert(sizeof(float) == 4);

    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));

    writeUint32LE(destination, bits);
}

float readFloat32LE(const uint8_t* source)
{
    const uint32_t bits = readUint32LE(source);

    float value;
    std::memcpy(&value, &bits, sizeof(value));

    return value;
}

} // namespace

bool encode(
    const Packet& packet,
    uint8_t* output,
    std::size_t outputSize)
{
    if (output == nullptr ||
        outputSize < kPacketSize)
    {
        return false;
    }

    // Sync
    writeUint16LE(output + 0, kSync);

    // Protocol version
    output[2] = kVersion;

    // Payload
    writeUint32LE(output + 3, packet.sessionId);
    writeUint16LE(output + 7, packet.sequence);

    writeFloat32LE(output + 9, packet.velocity);
    writeFloat32LE(output + 13, packet.steering);

    output[17] = packet.flags;

    // CRC covers bytes 2..17:
    // version + sessionId + sequence +
    // velocity + steering + flags.
    const uint16_t crc =
        crc16CcittFalse(
            output + 2,
            16);

    writeUint16LE(output + 18, crc);

    return true;
}

bool decode(
    const uint8_t* input,
    std::size_t inputSize,
    Packet& packet)
{
    if (input == nullptr ||
        inputSize < kPacketSize)
    {
        return false;
    }

    // Check sync.
    if (readUint16LE(input + 0) != kSync)
    {
        return false;
    }

    // Check protocol version.
    if (input[2] != kVersion)
    {
        return false;
    }

    // Check CRC over bytes 2..17.
    const uint16_t receivedCrc =
        readUint16LE(input + 18);

    const uint16_t calculatedCrc =
        crc16CcittFalse(
            input + 2,
            16);

    if (receivedCrc != calculatedCrc)
    {
        return false;
    }

    // Decode payload only after the packet
    // has passed all structural checks.
    packet.sessionId = readUint32LE(input + 3);
    packet.sequence = readUint16LE(input + 7);

    packet.velocity = readFloat32LE(input + 9);
    packet.steering = readFloat32LE(input + 13);

    packet.flags = input[17];

    return true;
}

} // namespace MotionCommandProtocol
