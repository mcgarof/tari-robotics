#include <unity.h>

#include "observability/ObservabilityDump.h"

void setUp()
{
}

void tearDown()
{
}

void test_initial_state_is_idle()
{
    ObservabilityDump dump;

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ObservabilityDumpState::IDLE),
        static_cast<int>(dump.state()));

    TEST_ASSERT_FALSE(dump.active());
}

void test_start_fast_dump()
{
    ObservabilityDump dump;

    TEST_ASSERT_TRUE(
        dump.start(ObservabilityDumpState::FAST));

    TEST_ASSERT_TRUE(dump.active());

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ObservabilityDumpState::FAST),
        static_cast<int>(dump.state()));
}

void test_cannot_start_another_dump_while_active()
{
    ObservabilityDump dump;

    TEST_ASSERT_TRUE(
        dump.start(ObservabilityDumpState::FAST));

    TEST_ASSERT_FALSE(
        dump.start(ObservabilityDumpState::EVENTS));

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ObservabilityDumpState::FAST),
        static_cast<int>(dump.state()));
}

void test_cannot_start_idle()
{
    ObservabilityDump dump;

    TEST_ASSERT_FALSE(
        dump.start(ObservabilityDumpState::IDLE));

    TEST_ASSERT_FALSE(dump.active());
}

void test_stop_returns_to_idle()
{
    ObservabilityDump dump;

    TEST_ASSERT_TRUE(
        dump.start(ObservabilityDumpState::SLOW));

    dump.stop();

    TEST_ASSERT_FALSE(dump.active());

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ObservabilityDumpState::IDLE),
        static_cast<int>(dump.state()));
}

void test_new_dump_can_start_after_stop()
{
    ObservabilityDump dump;

    TEST_ASSERT_TRUE(
        dump.start(ObservabilityDumpState::FAST));

    dump.stop();

    TEST_ASSERT_TRUE(
        dump.start(ObservabilityDumpState::EVENTS));

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ObservabilityDumpState::EVENTS),
        static_cast<int>(dump.state()));
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_initial_state_is_idle);
    RUN_TEST(test_start_fast_dump);
    RUN_TEST(test_cannot_start_another_dump_while_active);
    RUN_TEST(test_cannot_start_idle);
    RUN_TEST(test_stop_returns_to_idle);
    RUN_TEST(test_new_dump_can_start_after_stop);

    return UNITY_END();
}
