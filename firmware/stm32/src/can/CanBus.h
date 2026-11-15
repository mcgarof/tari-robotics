#pragma once

#include <cstddef>
#include <cstdint>

struct CanFrame
{
    uint32_t id;
    uint8_t data[8];
    std::size_t length;
};

class CanBus
{
public:
    virtual ~CanBus() = default;

    virtual bool send(const CanFrame& frame) = 0;
    virtual bool receive(CanFrame& frame) = 0;
};
