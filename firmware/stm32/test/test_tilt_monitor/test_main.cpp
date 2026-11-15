#include <unity.h>

#include "../../src/safety/TiltMonitor.h"

#include <cmath>
#include <limits>

namespace
{
constexpr float kPi = 3.14159265358979323846f;

constexpr float degreesToRadians(float degrees)
{
    return degrees * kPi / 180.0f;
}

constexpr float kFallThreshold = degreesToRadians(40.0f);
constexpr float kRearmThreshold = degreesToRadians(30.0f);
}

void test_tilt_below_fall_threshold_is_safe()
{
    TiltMonitor monitor(kFallThreshold, kRearmThreshold);

    TEST_ASSERT_FALSE(monitor.update(degreesToRadians(20.0f)));
}

void test_positive_tilt_at_fall_threshold_is_exceeded()
{
    TiltMonitor monitor(kFallThreshold, kRearmThreshold);

    TEST_ASSERT_TRUE(monitor.update(degreesToRadians(40.0f)));
}

void test_negative_tilt_at_fall_threshold_is_exceeded()
{
    TiltMonitor monitor(kFallThreshold, kRearmThreshold);

    TEST_ASSERT_TRUE(monitor.update(degreesToRadians(-40.0f)));
}

void test_tilt_remains_exceeded_inside_hysteresis_region()
{
    TiltMonitor monitor(kFallThreshold, kRearmThreshold);

    TEST_ASSERT_TRUE(monitor.update(degreesToRadians(45.0f)));
    TEST_ASSERT_TRUE(monitor.update(degreesToRadians(35.0f)));
}

void test_tilt_clears_at_rearm_threshold()
{
    TiltMonitor monitor(kFallThreshold, kRearmThreshold);

    TEST_ASSERT_TRUE(monitor.update(degreesToRadians(45.0f)));
    TEST_ASSERT_FALSE(monitor.update(degreesToRadians(30.0f)));
}

void test_non_finite_pitch_is_exceeded()
{
    TiltMonitor monitor(kFallThreshold, kRearmThreshold);

    TEST_ASSERT_TRUE(
        monitor.update(std::numeric_limits<float>::quiet_NaN()));

    TEST_ASSERT_TRUE(
        monitor.update(std::numeric_limits<float>::infinity()));

    TEST_ASSERT_TRUE(
        monitor.update(-std::numeric_limits<float>::infinity()));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_tilt_below_fall_threshold_is_safe);
    RUN_TEST(test_positive_tilt_at_fall_threshold_is_exceeded);
    RUN_TEST(test_negative_tilt_at_fall_threshold_is_exceeded);
    RUN_TEST(test_tilt_remains_exceeded_inside_hysteresis_region);
    RUN_TEST(test_tilt_clears_at_rearm_threshold);
    RUN_TEST(test_non_finite_pitch_is_exceeded);

    return UNITY_END();
}