#pragma once

#include <cstddef>
#include <cstdint>

namespace MotionCommandProtocol
{

constexpr uint16_t kSync = 0xA55A;
constexpr uint8_t kVersion = 1;

constexpr std::size_t kPacketSize = 20;

struct Packet
{
    uint32_t sessionId;
    uint16_t sequence;

    float velocity;   // m/s
    float steering;   // rad/s

    uint8_t flags;
};

bool encode(
    const Packet& packet,
    uint8_t* output,
    std::size_t outputSize);

bool decode(
    const uint8_t* input,
    std::size_t inputSize,
    Packet& packet);

} // namespace MotionCommandProtocol
