#pragma once

class TiltMonitor
{
public:
    TiltMonitor(float fallThresholdRad,
                float rearmThresholdRad);

    bool update(float pitchRad);

private:
    float fallThresholdRad_;
    float rearmThresholdRad_;
    bool exceeded_ = false;
};
