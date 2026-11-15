
#include <unity.h>
#include <limits>

#include "control/ControlParamsValidator.h"

namespace
{
ControlParams validParams()
{
    ControlParams params{};
    params.wheelRadius = 0.08255f;
    params.maxTorque = 5.0f;
    params.maxSpeedIntegralTorque = 1.0f;
    return params;
}
}

void test_valid_params()
{
    const auto params = validParams();

    TEST_ASSERT_EQUAL(
        static_cast<int>(ControlParamsValidationError::NONE),
        static_cast<int>(validateControlParams(params)));
}

void test_non_finite_params()
{
    auto params = validParams();
    params.kpBalance =
        std::numeric_limits<float>::quiet_NaN();

    TEST_ASSERT_EQUAL(
        static_cast<int>(ControlParamsValidationError::NON_FINITE),
        static_cast<int>(validateControlParams(params)));

    params = validParams();
    params.maxTorque =
        std::numeric_limits<float>::infinity();

    TEST_ASSERT_EQUAL(
        static_cast<int>(ControlParamsValidationError::NON_FINITE),
        static_cast<int>(validateControlParams(params)));
}

void test_out_of_range_params()
{
    auto params = validParams();
    params.wheelRadius = 0.0f;

    TEST_ASSERT_EQUAL(
        static_cast<int>(ControlParamsValidationError::OUT_OF_RANGE),
        static_cast<int>(validateControlParams(params)));

    params = validParams();
    params.maxTorque = -1.0f;

    TEST_ASSERT_EQUAL(
        static_cast<int>(ControlParamsValidationError::OUT_OF_RANGE),
        static_cast<int>(validateControlParams(params)));

    params = validParams();
    params.maxSpeedIntegralTorque = -0.1f;

    TEST_ASSERT_EQUAL(
        static_cast<int>(ControlParamsValidationError::OUT_OF_RANGE),
        static_cast<int>(validateControlParams(params)));
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_valid_params);
    RUN_TEST(test_non_finite_params);
    RUN_TEST(test_out_of_range_params);

    return UNITY_END();
}
