#pragma once

#include <algorithm>
#include <cstddef>

class NonBlockingWriter
{
public:
    bool begin(const char* data, std::size_t length)
    {
        if (busy() || data == nullptr || length == 0)
            return false;

        data_ = data;
        length_ = length;
        offset_ = 0;
        complete_ = false;

        return true;
    }

    template <typename Transport>
    void update(Transport& transport)
    {
        if (!busy())
            return;

        const std::size_t available =
            static_cast<std::size_t>(
                transport.availableForWrite());

        if (available == 0)
            return;

        const std::size_t remaining =
            length_ - offset_;

        const std::size_t requested =
            std::min(available, remaining);

        const std::size_t written =
            transport.write(
                reinterpret_cast<const uint8_t*>(
                    data_ + offset_),
                requested);

        offset_ += written;

        if (offset_ == length_)
        {
            data_ = nullptr;
            length_ = 0;
            offset_ = 0;
            complete_ = true;
        }
    }

    bool busy() const
    {
        return data_ != nullptr;
    }

    bool complete() const
    {
        return complete_;
    }

    void reset()
    {
        data_ = nullptr;
        length_ = 0;
        offset_ = 0;
        complete_ = false;
    }

private:
    const char* data_ = nullptr;

    std::size_t length_ = 0;
    std::size_t offset_ = 0;

    bool complete_ = false;
};
