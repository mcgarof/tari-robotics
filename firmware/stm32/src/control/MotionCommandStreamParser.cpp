#include "MotionCommandStreamParser.h"

bool MotionCommandStreamParser::pushByte(
    uint8_t byte,
    MotionCommandProtocol::Packet& packet)
{
    // Search for the first sync byte: 0x5A.
    if (size_ == 0)
    {
        if (byte == 0x5A)
        {
            buffer_[0] = byte;
            size_ = 1;
        }

        return false;
    }

    // We have 0x5A and are waiting for 0xA5.
    if (size_ == 1)
    {
        if (byte == 0xA5)
        {
            buffer_[1] = byte;
            size_ = 2;
        }
        else if (byte == 0x5A)
        {
            // This byte could itself be the beginning of a sync.
            buffer_[0] = byte;
        }
        else
        {
            size_ = 0;
        }

        return false;
    }

    buffer_[size_++] = byte;

    if (size_ < MotionCommandProtocol::kPacketSize)
    {
        return false;
    }

    if (MotionCommandProtocol::decode(
            buffer_,
            MotionCommandProtocol::kPacketSize,
            packet))
    {
        size_ = 0;
        return true;
    }

    // The candidate frame was invalid. Some bytes already received
    // could contain the beginning of the next valid frame.
    discardUntilPossibleSync();

    return false;
}

void MotionCommandStreamParser::reset()
{
    size_ = 0;
}

void MotionCommandStreamParser::discardUntilPossibleSync()
{
    constexpr uint8_t kSync0 = 0x5A;
    constexpr uint8_t kSync1 = 0xA5;

    for (std::size_t i = 1; i < size_; ++i)
    {
        if (buffer_[i] != kSync0)
        {
            continue;
        }

        // Full sync found inside the rejected candidate.
        if ((i + 1 < size_) &&
            (buffer_[i + 1] == kSync1))
        {
            const std::size_t remaining = size_ - i;

            for (std::size_t j = 0; j < remaining; ++j)
            {
                buffer_[j] = buffer_[i + j];
            }

            size_ = remaining;
            return;
        }

        // Last byte could be the first half of a future sync.
        if (i == size_ - 1)
        {
            buffer_[0] = kSync0;
            size_ = 1;
            return;
        }
    }

    size_ = 0;
}
