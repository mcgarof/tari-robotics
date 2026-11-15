#include <unity.h>

#include "control/ControlWatchdog.h"

void test_normal_timing_is_ok()
{
    ControlWatchdog watchdog(2000, 10000, 3);

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(watchdog.update(1000)));
}

void test_single_overrun_is_tolerated()
{
    ControlWatchdog watchdog(2000, 10000, 3);

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(watchdog.update(2300)));
}

void test_three_consecutive_overruns_become_unavailable()
{
    ControlWatchdog watchdog(2000, 10000, 3);

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(watchdog.update(2300)));

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(watchdog.update(2400)));

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::UNAVAILABLE),
        static_cast<int>(watchdog.update(2200)));
}

void test_normal_cycle_clears_overrun_count()
{
    ControlWatchdog watchdog(2000, 10000, 3);

    watchdog.update(2300);
    watchdog.update(2400);

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(watchdog.update(1000)));

    // We must start counting again from zero.
    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(watchdog.update(2300)));

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(watchdog.update(2400)));
}

void test_severe_overrun_is_immediately_unavailable()
{
    ControlWatchdog watchdog(2000, 10000, 3);

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::UNAVAILABLE),
        static_cast<int>(watchdog.update(1000000)));
}

void test_overrun_threshold_itself_is_accepted()
{
    ControlWatchdog watchdog(2000, 10000, 3);

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(watchdog.update(2000)));
}

void test_lost_threshold_itself_is_not_immediate_failure()
{
    ControlWatchdog watchdog(2000, 10000, 3);

    // 10000 us is an overrun, but not yet a severe/lost cycle.
    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(watchdog.update(10000)));
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_normal_timing_is_ok);
    RUN_TEST(test_single_overrun_is_tolerated);
    RUN_TEST(test_three_consecutive_overruns_become_unavailable);
    RUN_TEST(test_normal_cycle_clears_overrun_count);
    RUN_TEST(test_severe_overrun_is_immediately_unavailable);
    RUN_TEST(test_overrun_threshold_itself_is_accepted);
    RUN_TEST(test_lost_threshold_itself_is_not_immediate_failure);

    UNITY_END();
}
