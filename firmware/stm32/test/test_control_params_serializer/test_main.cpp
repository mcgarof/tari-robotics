#include <unity.h>

#include <cstring>

#include "../../src/control/ControlParamsSerializer.h"

void test_serialize_control_params()
{
    ControlParams params{};

    params.kpBalance = 12.5f;
    params.kdBalance = 0.8f;
    params.wheelRadius = 0.08255f;
    params.maxTorque = 5.0f;

    char buffer[256]{};

    const std::size_t length =
        serializeControlParams(
            params, buffer, sizeof(buffer));

    TEST_ASSERT_GREATER_THAN(0, length);
    TEST_ASSERT_EQUAL(strlen(buffer), length);

    TEST_ASSERT_NOT_NULL(
        strstr(buffer, "kpBalance=12.500000"));

    TEST_ASSERT_NOT_NULL(
        strstr(buffer, "wheelRadius=0.082550"));

    TEST_ASSERT_NOT_NULL(
        strstr(buffer, "maxTorque=5.000000"));

    TEST_ASSERT_EQUAL('\n', buffer[length - 1]);
}

void test_serialize_control_params_small_buffer()
{
    ControlParams params{};

    char buffer[16]{};

    const std::size_t length =
        serializeControlParams(
            params, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL(0, length);
    TEST_ASSERT_EQUAL('\0', buffer[0]);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_serialize_control_params);
    RUN_TEST(test_serialize_control_params_small_buffer);

    return UNITY_END();
}
