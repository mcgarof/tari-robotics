#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

#include <unity.h>

#include "observability/FastFlightSample.h"
#include "observability/FlightDumpPublisher.h"
#include "observability/FlightRecorder.h"
#include "observability/FlightRecorderSerializer.h"

class FakeTransport
{
public:
    explicit FakeTransport(std::size_t available)
        : available_(available)
    {
    }

    int availableForWrite() const
    {
        return static_cast<int>(available_);
    }

    std::size_t write(
        const uint8_t* data,
        std::size_t length)
    {
        const std::size_t count =
            std::min(available_, length);

        output_.append(
            reinterpret_cast<const char*>(data),
            count);

        return count;
    }

    const std::string& output() const
    {
        return output_;
    }

private:
    std::size_t available_;
    std::string output_;
};

void setUp()
{
}

void tearDown()
{
}

void test_fast_dump_is_incremental_and_ordered()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    FastFlightSample first{};
    first.pitch = 100;

    FastFlightSample second{};
    second.pitch = 200;

    recorder.record(1000U, first);
    recorder.record(2000U, second);

    recorder.freeze();

    FlightDumpPublisher<FastFlightSample, 3> dump{
        serializeFastFlightSample};

    FakeTransport transport{5};

    TEST_ASSERT_TRUE(dump.start());
    TEST_ASSERT_TRUE(dump.active());

    // Five bytes per update are intentionally too few for
    // a complete CSV row, forcing incremental transmission.
    while (dump.active())
    {
        dump.update(recorder, transport);
    }

    TEST_ASSERT_EQUAL_STRING(
        "F,1,1000,100,0,0,0,0,0,0,0\n"
        "F,1,2000,200,0,0,0,0,0,0,0\n",
        transport.output().c_str());
}

void test_empty_recorder_completes_without_output()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    recorder.freeze();

    FlightDumpPublisher<FastFlightSample, 3> dump{
        serializeFastFlightSample};

    FakeTransport transport{5};

    TEST_ASSERT_TRUE(dump.start());
    TEST_ASSERT_TRUE(dump.active());

    dump.update(recorder, transport);

    TEST_ASSERT_FALSE(dump.active());
    TEST_ASSERT_TRUE(transport.output().empty());
}

void test_slow_dump_uses_same_publisher()
{
    FlightRecorder<SlowFlightSample, 3> recorder;

    SlowFlightSample first{};
    first.pitch = 100;
    first.commandVelocity = 500;
    first.speedIntegralTorque = 1200;
    first.robotMode = 2;
    first.statusFlags = 5;

    SlowFlightSample second{};
    second.pitch = 200;
    second.commandVelocity = -300;
    second.speedIntegralTorque = -800;
    second.robotMode = 3;
    second.statusFlags = 9;

    recorder.record(1000U, first);
    recorder.record(2000U, second);

    recorder.freeze();

    FlightDumpPublisher<SlowFlightSample, 3> dump{
        serializeSlowFlightSample};

    FakeTransport transport{5};

    TEST_ASSERT_TRUE(dump.start());

    while (dump.active())
    {
        dump.update(recorder, transport);
    }

    TEST_ASSERT_EQUAL_STRING(
        "S,1,1000,100,0,0,0,0,0,500,0,1200,2,5\n"
        "S,1,2000,200,0,0,0,0,0,-300,0,-800,3,9\n",
        transport.output().c_str());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_fast_dump_is_incremental_and_ordered);
    RUN_TEST(test_empty_recorder_completes_without_output);
    RUN_TEST(test_slow_dump_uses_same_publisher);

    return UNITY_END();
}
