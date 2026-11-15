
#include <unity.h>

#include "motor/TorqueLimiter.h"

void test_torque_within_limits()
{
    const auto result =
        limitTorque(2.0f, -3.0f, 5.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 2.0f, result.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, -3.0f, result.rightTorque);

    TEST_ASSERT_FALSE(result.clamped);
}

void test_torque_above_limits()
{
    const auto result =
        limitTorque(7.0f, -8.0f, 5.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 5.0f, result.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, -5.0f, result.rightTorque);

    TEST_ASSERT_TRUE(result.clamped);
}

void test_only_one_motor_clamped()
{
    const auto result =
        limitTorque(2.0f, -6.0f, 5.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 2.0f, result.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, -5.0f, result.rightTorque);

    TEST_ASSERT_TRUE(result.clamped);
}

void test_torque_at_exact_limit()
{
    const auto result =
        limitTorque(5.0f, -5.0f, 5.0f);

    TEST_ASSERT_FALSE(result.clamped);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_torque_within_limits);
    RUN_TEST(test_torque_above_limits);
    RUN_TEST(test_only_one_motor_clamped);
    RUN_TEST(test_torque_at_exact_limit);

    return UNITY_END();
}
