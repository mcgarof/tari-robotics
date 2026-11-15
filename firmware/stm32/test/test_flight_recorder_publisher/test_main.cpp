#include <unity.h>

#include "observability/FastFlightSample.h"
#include "observability/FlightRecorder.h"
#include "observability/FlightRecorderPublisher.h"

namespace
{
using Recorder = FlightRecorder<FastFlightSample, 3>;

FastFlightSample makeSample(int16_t pitch)
{
    FastFlightSample sample{};
    sample.pitch = pitch;
    return sample;
}
}

void setUp()
{
}

void tearDown()
{
}

void test_empty_recorder_has_no_sample()
{
    Recorder recorder;
    FlightRecorderPublisher publisher;

    FastFlightSample sample{};

    TEST_ASSERT_FALSE(
        publisher.next(recorder, sample));
}

void test_next_does_not_advance()
{
    Recorder recorder;
    FlightRecorderPublisher publisher;

    recorder.record(1000U, makeSample(10));
    recorder.record(2000U, makeSample(20));

    FastFlightSample sample{};

    TEST_ASSERT_TRUE(
        publisher.next(recorder, sample));

    TEST_ASSERT_EQUAL_UINT32(1000U, sample.timestampUs);
    TEST_ASSERT_EQUAL_INT16(10, sample.pitch);

    // Calling next() again without markPublished()
    // must return the same sample.
    TEST_ASSERT_TRUE(
        publisher.next(recorder, sample));

    TEST_ASSERT_EQUAL_UINT32(1000U, sample.timestampUs);
    TEST_ASSERT_EQUAL_INT16(10, sample.pitch);
}

void test_mark_published_advances()
{
    Recorder recorder;
    FlightRecorderPublisher publisher;

    recorder.record(1000U, makeSample(10));
    recorder.record(2000U, makeSample(20));

    FastFlightSample sample{};

    TEST_ASSERT_TRUE(
        publisher.next(recorder, sample));

    TEST_ASSERT_EQUAL_UINT32(1000U, sample.timestampUs);
    TEST_ASSERT_EQUAL_INT16(10, sample.pitch);

    publisher.markPublished();

    TEST_ASSERT_TRUE(
        publisher.next(recorder, sample));

    TEST_ASSERT_EQUAL_UINT32(2000U, sample.timestampUs);
    TEST_ASSERT_EQUAL_INT16(20, sample.pitch);
}

void test_no_sample_after_last_published()
{
    Recorder recorder;
    FlightRecorderPublisher publisher;

    recorder.record(1000U, makeSample(10));

    FastFlightSample sample{};

    TEST_ASSERT_TRUE(
        publisher.next(recorder, sample));

    publisher.markPublished();

    TEST_ASSERT_FALSE(
        publisher.next(recorder, sample));
}

void test_reset_restarts_from_oldest()
{
    Recorder recorder;
    FlightRecorderPublisher publisher;

    recorder.record(1000U, makeSample(10));
    recorder.record(2000U, makeSample(20));

    FastFlightSample sample{};

    TEST_ASSERT_TRUE(
        publisher.next(recorder, sample));

    publisher.markPublished();

    TEST_ASSERT_TRUE(
        publisher.next(recorder, sample));

    TEST_ASSERT_EQUAL_UINT32(2000U, sample.timestampUs);
    TEST_ASSERT_EQUAL_INT16(20, sample.pitch);

    publisher.reset();

    TEST_ASSERT_TRUE(
        publisher.next(recorder, sample));

    TEST_ASSERT_EQUAL_UINT32(1000U, sample.timestampUs);
    TEST_ASSERT_EQUAL_INT16(10, sample.pitch);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_empty_recorder_has_no_sample);
    RUN_TEST(test_next_does_not_advance);
    RUN_TEST(test_mark_published_advances);
    RUN_TEST(test_no_sample_after_last_published);
    RUN_TEST(test_reset_restarts_from_oldest);

    return UNITY_END();
}
