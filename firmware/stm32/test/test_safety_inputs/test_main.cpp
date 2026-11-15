#include <unity.h>

#include "../../src/safety/SafetyInputs.h"

void test_inputs_are_inactive_at_startup()
{
    SafetyInputs inputs;

    TEST_ASSERT_FALSE(inputs.enableRequested());
    TEST_ASSERT_FALSE(inputs.resetRequested());
}

void test_enable_follows_debounced_input_level()
{
    SafetyInputs inputs;

    // ENABLE goes HIGH: debounce starts.
    inputs.update(true, false, 0);
    TEST_ASSERT_FALSE(inputs.enableRequested());

    // Still HIGH, but debounce interval has not elapsed yet.
    inputs.update(true, false, 29);
    TEST_ASSERT_FALSE(inputs.enableRequested());

    // HIGH has now been stable for 30 ms.
    inputs.update(true, false, 30);
    TEST_ASSERT_TRUE(inputs.enableRequested());

    // Disabling must be immediate (fail-safe).
    inputs.update(false, false, 31);
    TEST_ASSERT_FALSE(inputs.enableRequested());
}

void test_reset_is_requested_after_debounce()
{
    SafetyInputs inputs;

    inputs.update(false, false, 0);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    // Rising edge starts debounce.
    inputs.update(false, true, 10);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    // Still below debounce threshold.
    inputs.update(false, true, 39);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    // HIGH has been stable for 30 ms.
    inputs.update(false, true, 40);
    TEST_ASSERT_TRUE(inputs.resetRequested());

    // Request lasts for one update only.
    inputs.update(false, true, 41);
    TEST_ASSERT_FALSE(inputs.resetRequested());
}

void test_reset_release_does_not_request_reset()
{
    SafetyInputs inputs;

    inputs.update(false, true, 0);
    inputs.update(false, false, 0);

    TEST_ASSERT_FALSE(inputs.resetRequested());
}

void test_second_reset_press_generates_new_request()
{
    SafetyInputs inputs;

    // First press.
    inputs.update(false, true, 0);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    inputs.update(false, true, 30);
    TEST_ASSERT_TRUE(inputs.resetRequested());

    // Release re-arms the RESET input.
    inputs.update(false, false, 31);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    // Second press starts a new debounce.
    inputs.update(false, true, 40);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    inputs.update(false, true, 70);
    TEST_ASSERT_TRUE(inputs.resetRequested());
}

void test_short_enable_pulse_does_not_request_enable()
{
    SafetyInputs inputs;

    inputs.update(false, false, 0);
    TEST_ASSERT_FALSE(inputs.enableRequested());

    // Contact closes, but only briefly.
    inputs.update(true, false, 10);
    TEST_ASSERT_FALSE(inputs.enableRequested());

    // It opens again before the debounce interval expires.
    inputs.update(false, false, 20);
    TEST_ASSERT_FALSE(inputs.enableRequested());
}

void test_short_reset_pulse_does_not_request_reset()
{
    SafetyInputs inputs;

    inputs.update(false, false, 0);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    // RESET goes HIGH: debounce should start.
    inputs.update(false, true, 10);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    // It returns LOW before the debounce interval expires.
    inputs.update(false, false, 20);
    TEST_ASSERT_FALSE(inputs.resetRequested());
}

void test_reset_is_not_repeated_while_input_remains_high()
{
    SafetyInputs inputs;

    // RESET goes HIGH: debounce starts.
    inputs.update(false, true, 0);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    // After 30 ms the press is accepted exactly once.
    inputs.update(false, true, 30);
    TEST_ASSERT_TRUE(inputs.resetRequested());

    // Keeping the button pressed must not generate more requests.
    inputs.update(false, true, 31);
    TEST_ASSERT_FALSE(inputs.resetRequested());

    inputs.update(false, true, 100);
    TEST_ASSERT_FALSE(inputs.resetRequested());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_inputs_are_inactive_at_startup);
    RUN_TEST(test_enable_follows_debounced_input_level);
    RUN_TEST(test_reset_is_requested_after_debounce);
    RUN_TEST(test_reset_release_does_not_request_reset);
    RUN_TEST(test_second_reset_press_generates_new_request);
    RUN_TEST(test_short_enable_pulse_does_not_request_enable);
    RUN_TEST(test_short_reset_pulse_does_not_request_reset);
    RUN_TEST(test_reset_is_not_repeated_while_input_remains_high);

    return UNITY_END();
}
