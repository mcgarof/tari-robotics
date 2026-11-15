#include <unity.h>

#include "../../src/control/CommandSequenceTracker.h"

void test_first_sequence_is_accepted()
{
    CommandSequenceTracker tracker;

    TEST_ASSERT_TRUE(
        tracker.accept(100, 42));
}

void test_newer_sequence_is_accepted()
{
    CommandSequenceTracker tracker;

    TEST_ASSERT_TRUE(tracker.accept(100, 42));
    TEST_ASSERT_TRUE(tracker.accept(100, 43));
    TEST_ASSERT_TRUE(tracker.accept(100, 44));
}

void test_duplicate_sequence_is_rejected()
{
    CommandSequenceTracker tracker;

    TEST_ASSERT_TRUE(tracker.accept(100, 42));
    TEST_ASSERT_FALSE(tracker.accept(100, 42));
}

void test_older_sequence_is_rejected()
{
    CommandSequenceTracker tracker;

    TEST_ASSERT_TRUE(tracker.accept(100, 42));
    TEST_ASSERT_FALSE(tracker.accept(100, 41));
}

void test_new_session_accepts_lower_sequence()
{
    CommandSequenceTracker tracker;

    TEST_ASSERT_TRUE(tracker.accept(100, 500));

    // New sender session: sequence may restart from zero.
    TEST_ASSERT_TRUE(tracker.accept(200, 0));
    TEST_ASSERT_TRUE(tracker.accept(200, 1));
}

void test_sequence_wraparound_is_accepted()
{
    CommandSequenceTracker tracker;

    TEST_ASSERT_TRUE(tracker.accept(100, 65534));
    TEST_ASSERT_TRUE(tracker.accept(100, 65535));
    TEST_ASSERT_TRUE(tracker.accept(100, 0));
    TEST_ASSERT_TRUE(tracker.accept(100, 1));
}

void test_old_sequence_after_wraparound_is_rejected()
{
    CommandSequenceTracker tracker;

    TEST_ASSERT_TRUE(tracker.accept(100, 65535));
    TEST_ASSERT_TRUE(tracker.accept(100, 0));

    // 65535 now belongs to the past.
    TEST_ASSERT_FALSE(tracker.accept(100, 65535));
}

void test_reset_accepts_next_sequence()
{
    CommandSequenceTracker tracker;

    TEST_ASSERT_TRUE(tracker.accept(100, 500));

    tracker.reset();

    TEST_ASSERT_TRUE(tracker.accept(100, 10));
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_first_sequence_is_accepted);
    RUN_TEST(test_newer_sequence_is_accepted);
    RUN_TEST(test_duplicate_sequence_is_rejected);
    RUN_TEST(test_older_sequence_is_rejected);
    RUN_TEST(test_new_session_accepts_lower_sequence);
    RUN_TEST(test_sequence_wraparound_is_accepted);
    RUN_TEST(test_old_sequence_after_wraparound_is_rejected);
    RUN_TEST(test_reset_accepts_next_sequence);

    return UNITY_END();
}
