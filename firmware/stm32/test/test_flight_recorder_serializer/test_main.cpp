#include <cstring>

#include <unity.h>

#include "observability/FlightRecorderSerializer.h"

void setUp()
{
}

void tearDown()
{
}

void test_serialize_fast_sample()
{
    FastFlightSample sample{};

    sample.timestampUs = 12345678U;
    sample.pitch = 125;
    sample.pitchRate = -34;
    sample.yaw = 87;
    sample.yawRate = 12;
    sample.leftWheelSpeed = 350;
    sample.rightWheelSpeed = 347;
    sample.leftTorque = 1250;
    sample.rightTorque = 1270;

    char buffer[128]{};

    const std::size_t length =
        serializeFastFlightSample(
            sample,
            buffer,
            sizeof(buffer));

    TEST_ASSERT_GREATER_THAN(0, length);

    TEST_ASSERT_EQUAL_STRING(
        "F,1,12345678,125,-34,87,12,350,347,1250,1270\n",
        buffer);

    TEST_ASSERT_EQUAL_UINT(
        std::strlen(buffer),
        length);
}

void test_buffer_too_small()
{
    FastFlightSample sample{};
    sample.timestampUs = 12345678U;

    char buffer[8]{};

    const std::size_t length =
        serializeFastFlightSample(
            sample,
            buffer,
            sizeof(buffer));

    TEST_ASSERT_EQUAL_UINT(0, length);
}

void test_null_buffer()
{
    FastFlightSample sample{};

    const std::size_t length =
        serializeFastFlightSample(
            sample,
            nullptr,
            128);

    TEST_ASSERT_EQUAL_UINT(0, length);
}

void test_zero_buffer_size()
{
    FastFlightSample sample{};
    char buffer[1]{};

    const std::size_t length =
        serializeFastFlightSample(
            sample,
            buffer,
            0);

    TEST_ASSERT_EQUAL_UINT(0, length);
}

void test_serialize_slow_sample()
{
    SlowFlightSample sample{};

    sample.timestampUs = 87654321U;
    sample.pitch = 100;
    sample.pitchRate = -20;
    sample.yaw = 300;
    sample.yawRate = -40;
    sample.leftWheelSpeed = 500;
    sample.rightWheelSpeed = 510;
    sample.commandVelocity = 750;
    sample.commandSteering = -250;
    sample.speedIntegralTorque = 1200;
    sample.robotMode = 2;
    sample.statusFlags = 0x35;

    char buffer[160]{};

    const std::size_t length =
        serializeSlowFlightSample(
            sample,
            buffer,
            sizeof(buffer));

    TEST_ASSERT_GREATER_THAN(0, length);

    TEST_ASSERT_EQUAL_STRING(
        "S,1,87654321,100,-20,300,-40,500,510,750,-250,1200,2,53\n",
        buffer);

    TEST_ASSERT_EQUAL_UINT(
        std::strlen(buffer),
        length);
}

void test_serialize_event()
{
    Event event{};

    event.timestampUs = 12345678U;
    event.sequence = 42U;
    event.id = EventId::COMMAND_TIMEOUT;
    event.severity = EventSeverity::WARNING;

    char buffer[128]{};

    const std::size_t length =
        serializeEvent(
            event,
            buffer,
            sizeof(buffer));

    TEST_ASSERT_GREATER_THAN(0, length);

    TEST_ASSERT_EQUAL_STRING(
        "E,1,12345678,42,3,1\n",
        buffer);

    TEST_ASSERT_EQUAL_UINT(
        std::strlen(buffer),
        length);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_serialize_fast_sample);
    RUN_TEST(test_buffer_too_small);
    RUN_TEST(test_null_buffer);
    RUN_TEST(test_zero_buffer_size);
    RUN_TEST(test_serialize_slow_sample);
    RUN_TEST(test_serialize_event);

    return UNITY_END();
}
