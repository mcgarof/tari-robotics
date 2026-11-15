#include "TiltMonitor.h"

#include <cmath>

TiltMonitor::TiltMonitor(float fallThresholdRad,
                         float rearmThresholdRad)
    : fallThresholdRad_(fallThresholdRad),
      rearmThresholdRad_(rearmThresholdRad)
{
}

bool TiltMonitor::update(float pitchRad)
{
    if (!std::isfinite(pitchRad))
    {
        exceeded_ = true;
        return exceeded_;
    }

    const float absolutePitch = std::fabs(pitchRad);

    if (!exceeded_)
    {
        if (absolutePitch >= fallThresholdRad_)
        {
            exceeded_ = true;
        }
    }
    else
    {
        if (absolutePitch <= rearmThresholdRad_)
        {
            exceeded_ = false;
        }
    }

    return exceeded_;
}
