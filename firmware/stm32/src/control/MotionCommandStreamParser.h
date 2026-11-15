#pragma once

#include "MotionCommandProtocol.h"

#include <cstddef>
#include <cstdint>

class MotionCommandStreamParser
{
public:
    bool pushByte(
        uint8_t byte,
        MotionCommandProtocol::Packet& packet);

    void reset();

private:
    uint8_t buffer_[MotionCommandProtocol::kPacketSize]{};
    std::size_t size_ = 0;

    void discardUntilPossibleSync();
};