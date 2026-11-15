#include <unity.h>

#include "control/MotionCommandValidator.h"

#include <limits>

namespace
{

MotionCommandValidator makeValidator()
{
    return MotionCommandValidator({
        .maxVelocity = 2.0f,
        .maxSteeringRate = 1.5f,
    });
}

MotionCommandProtocol::Packet makePacket(
    float velocity,
    float steering)
{
    return {
        .sessionId = 1,
        .sequence = 1,
        .velocity = velocity,
        .steering = steering,
        .flags = 0,
    };
}

void test_zero_command_is_valid()
{
    const auto validator = makeValidator();

    TEST_ASSERT_TRUE(
        validator.valid(makePacket(0.0f, 0.0f)));
}

void test_values_at_limits_are_valid()
{
    const auto validator = makeValidator();

    TEST_ASSERT_TRUE(
        validator.valid(makePacket(2.0f, 1.5f)));

    TEST_ASSERT_TRUE(
        validator.valid(makePacket(-2.0f, -1.5f)));
}

void test_velocity_above_limit_is_invalid()
{
    const auto validator = makeValidator();

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(2.01f, 0.0f)));

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(-2.01f, 0.0f)));
}

void test_steering_above_limit_is_invalid()
{
    const auto validator = makeValidator();

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(0.0f, 1.51f)));

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(0.0f, -1.51f)));
}

void test_nan_is_invalid()
{
    const auto validator = makeValidator();
    const float nan =
        std::numeric_limits<float>::quiet_NaN();

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(nan, 0.0f)));

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(0.0f, nan)));
}

void test_infinity_is_invalid()
{
    const auto validator = makeValidator();

    const float infinity =
        std::numeric_limits<float>::infinity();

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(infinity, 0.0f)));

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(-infinity, 0.0f)));

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(0.0f, infinity)));

    TEST_ASSERT_FALSE(
        validator.valid(makePacket(0.0f, -infinity)));
}

}

int main(int, char**)
{
    UNITY_BEGIN();

    RUN_TEST(test_zero_command_is_valid);
    RUN_TEST(test_values_at_limits_are_valid);
    RUN_TEST(test_velocity_above_limit_is_invalid);
    RUN_TEST(test_steering_above_limit_is_invalid);
    RUN_TEST(test_nan_is_invalid);
    RUN_TEST(test_infinity_is_invalid);

    return UNITY_END();
}
