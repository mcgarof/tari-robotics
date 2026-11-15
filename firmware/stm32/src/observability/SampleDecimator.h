#pragma once

#include <cstdint>

class SampleDecimator
{
public:
    explicit SampleDecimator(uint8_t factor)
        : factor_(factor)
    {
    }

    bool tick()
    {
        ++counter_;

        if (counter_ < factor_)
            return false;

        counter_ = 0;
        return true;
    }

    void reset()
    {
        counter_ = 0;
    }

private:
    uint8_t factor_;
    uint8_t counter_ = 0;
};
