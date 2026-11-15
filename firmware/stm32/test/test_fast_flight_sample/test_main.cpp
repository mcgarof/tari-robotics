#include <unity.h>

#include <cmath>
#include <limits>

#include "observability/FastFlightSample.h"

void test_encode_normal_values()
{
    TEST_ASSERT_EQUAL_INT16(
        1234,
        encodeFlightValue(1.234f, 1000.0f));

    TEST_ASSERT_EQUAL_INT16(
        -1234,
        encodeFlightValue(-1.234f, 1000.0f));

    TEST_ASSERT_EQUAL_INT16(
        0,
        encodeFlightValue(0.0f, 1000.0f));
}

void test_encode_saturates_positive()
{
    TEST_ASSERT_EQUAL_INT16(
        kMaxFlightValue,
        encodeFlightValue(100.0f, 1000.0f));
}

void test_encode_saturates_negative()
{
    TEST_ASSERT_EQUAL_INT16(
        kMinFlightValue,
        encodeFlightValue(-100.0f, 1000.0f));
}

void test_encode_invalid_values()
{
    TEST_ASSERT_EQUAL_INT16(
        kInvalidFlightValue,
        encodeFlightValue(
            std::numeric_limits<float>::quiet_NaN(),
            1000.0f));

    TEST_ASSERT_EQUAL_INT16(
        kInvalidFlightValue,
        encodeFlightValue(
            std::numeric_limits<float>::infinity(),
            1000.0f));

    TEST_ASSERT_EQUAL_INT16(
        kInvalidFlightValue,
        encodeFlightValue(
            -std::numeric_limits<float>::infinity(),
            1000.0f));
}

int main(int, char**)
{
    UNITY_BEGIN();

    RUN_TEST(test_encode_normal_values);
    RUN_TEST(test_encode_saturates_positive);
    RUN_TEST(test_encode_saturates_negative);
    RUN_TEST(test_encode_invalid_values);

    return UNITY_END();
}
