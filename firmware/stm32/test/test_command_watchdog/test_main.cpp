#include <unity.h>

#include "../../src/control/CommandWatchdog.h"

void test_command_is_not_fresh_before_first_command()
{
    CommandWatchdog watchdog(100);

    TEST_ASSERT_FALSE(watchdog.commandFresh(1000));
}

void test_command_is_fresh_after_receive()
{
    CommandWatchdog watchdog(100);

    watchdog.notifyCommandReceived(1000);

    TEST_ASSERT_TRUE(watchdog.commandFresh(1000));
    TEST_ASSERT_TRUE(watchdog.commandFresh(1050));
    TEST_ASSERT_TRUE(watchdog.commandFresh(1100));
}

void test_command_becomes_stale_after_timeout()
{
    CommandWatchdog watchdog(100);

    watchdog.notifyCommandReceived(1000);

    TEST_ASSERT_FALSE(watchdog.commandFresh(1101));
}

void test_reset_makes_command_stale()
{
    CommandWatchdog watchdog(100);

    watchdog.notifyCommandReceived(1000);
    TEST_ASSERT_TRUE(watchdog.commandFresh(1000));

    watchdog.reset();

    TEST_ASSERT_FALSE(watchdog.commandFresh(1000));
}

void test_command_freshness_handles_uint32_wraparound()
{
    CommandWatchdog watchdog(100);

    watchdog.notifyCommandReceived(0xFFFFFFF0);

    // 0xFFFFFFF0 -> 0x00000010 = 32 ms
    TEST_ASSERT_TRUE(
        watchdog.commandFresh(0x00000010));

    // 0xFFFFFFF0 -> 0x00000060 = 112 ms
    TEST_ASSERT_FALSE(
        watchdog.commandFresh(0x00000060));
}

void test_command_received_tracks_whether_a_command_was_seen()
{
    CommandWatchdog watchdog(200);

    TEST_ASSERT_FALSE(watchdog.commandReceived());

    watchdog.notifyCommandReceived(1000);

    TEST_ASSERT_TRUE(watchdog.commandReceived());

    watchdog.reset();

    TEST_ASSERT_FALSE(watchdog.commandReceived());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_command_is_not_fresh_before_first_command);
    RUN_TEST(test_command_is_fresh_after_receive);
    RUN_TEST(test_command_becomes_stale_after_timeout);
    RUN_TEST(test_reset_makes_command_stale);
    RUN_TEST(test_command_freshness_handles_uint32_wraparound);
    RUN_TEST(test_command_received_tracks_whether_a_command_was_seen);

    return UNITY_END();
}
