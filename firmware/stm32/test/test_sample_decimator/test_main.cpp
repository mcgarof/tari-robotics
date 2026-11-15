#include <unity.h>

#include "observability/SampleDecimator.h"

void test_decimates_by_factor()
{
    SampleDecimator decimator{5};

    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());

    TEST_ASSERT_TRUE(decimator.tick());

    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());

    TEST_ASSERT_TRUE(decimator.tick());
}

void test_reset_restarts_decimation()
{
    SampleDecimator decimator{5};

    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());

    decimator.reset();

    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());
    TEST_ASSERT_FALSE(decimator.tick());

    TEST_ASSERT_TRUE(decimator.tick());
}

int main(int, char**)
{
    UNITY_BEGIN();

    RUN_TEST(test_decimates_by_factor);
    RUN_TEST(test_reset_restarts_decimation);

    return UNITY_END();
}
