#include <unity.h>

#include "observability/FlightRecorder.h"
#include "observability/FastFlightSample.h"

FastFlightSample makeSample(int16_t pitch)
{
    FastFlightSample sample{};
    sample.pitch = pitch;
    return sample;
}

void test_starts_empty()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    TEST_ASSERT_TRUE(recorder.empty());
    TEST_ASSERT_EQUAL_UINT32(0, recorder.size());
    TEST_ASSERT_EQUAL_UINT32(3, recorder.capacity());
}

void test_records_samples_in_order()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    recorder.record(1000, makeSample(10));
    recorder.record(2000, makeSample(20));

    TEST_ASSERT_EQUAL_UINT32(2, recorder.size());

    FastFlightSample sample{};

    TEST_ASSERT_TRUE(recorder.get(0, sample));
    TEST_ASSERT_EQUAL_INT16(10, sample.pitch);

    TEST_ASSERT_TRUE(recorder.get(1, sample));
    TEST_ASSERT_EQUAL_INT16(20, sample.pitch);
}

void test_overwrites_oldest_sample()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    recorder.record(1000, makeSample(10));
    recorder.record(2000, makeSample(20));
    recorder.record(3000, makeSample(30));
    recorder.record(4000, makeSample(40));

    TEST_ASSERT_EQUAL_UINT32(3, recorder.size());

    FastFlightSample sample{};

    TEST_ASSERT_TRUE(recorder.get(0, sample));
    TEST_ASSERT_EQUAL_INT16(20, sample.pitch);

    TEST_ASSERT_TRUE(recorder.get(1, sample));
    TEST_ASSERT_EQUAL_INT16(30, sample.pitch);

    TEST_ASSERT_TRUE(recorder.get(2, sample));
    TEST_ASSERT_EQUAL_INT16(40, sample.pitch);
}

void test_invalid_get_leaves_output_unchanged()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    recorder.record(1000, makeSample(10));

    FastFlightSample sample = makeSample(99);

    TEST_ASSERT_FALSE(recorder.get(1, sample));

    TEST_ASSERT_EQUAL_INT16(99, sample.pitch);
}

void test_clear()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    recorder.record(1000, makeSample(10));
    recorder.record(2000, makeSample(20));

    recorder.clear();

    TEST_ASSERT_TRUE(recorder.empty());
    TEST_ASSERT_EQUAL_UINT32(0, recorder.size());
}

void test_freeze_stops_recording()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    recorder.record(1000, makeSample(10));
    recorder.record(2000, makeSample(20));

    recorder.freeze();

    TEST_ASSERT_TRUE(recorder.frozen());

    recorder.record(3000, makeSample(30));

    TEST_ASSERT_EQUAL_UINT32(2, recorder.size());

    FastFlightSample sample{};

    TEST_ASSERT_TRUE(recorder.get(1, sample));
    TEST_ASSERT_EQUAL_INT16(20, sample.pitch);
}

void test_unfreeze_resumes_recording()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    recorder.record(1000, makeSample(10));

    recorder.freeze();
    recorder.record(2000, makeSample(20));

    recorder.unfreeze();

    TEST_ASSERT_FALSE(recorder.frozen());

    recorder.record(3000, makeSample(30));

    TEST_ASSERT_EQUAL_UINT32(2, recorder.size());

    FastFlightSample sample{};

    TEST_ASSERT_TRUE(recorder.get(1, sample));
    TEST_ASSERT_EQUAL_INT16(30, sample.pitch);
}

void test_clear_does_not_unfreeze()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    recorder.record(1000, makeSample(10));

    recorder.freeze();
    recorder.clear();

    TEST_ASSERT_TRUE(recorder.frozen());
    TEST_ASSERT_TRUE(recorder.empty());

    recorder.record(2000, makeSample(20));

    TEST_ASSERT_TRUE(recorder.empty());
}

void test_record_stores_timestamp()
{
    FlightRecorder<FastFlightSample, 3> recorder;

    recorder.record(123456U, makeSample(42));

    FastFlightSample sample{};
    TEST_ASSERT_TRUE(recorder.get(0, sample));

    TEST_ASSERT_EQUAL_UINT32(123456U, sample.timestampUs);
    TEST_ASSERT_EQUAL_INT16(42, sample.pitch);
}

void test_rearm_clears_and_unfreezes_recorder()
{
    FlightRecorder<FastFlightSample, 4> recorder;

    FastFlightSample sample{};

    recorder.record(1000U, sample);
    recorder.record(2000U, sample);

    recorder.freeze();

    TEST_ASSERT_TRUE(recorder.frozen());
    TEST_ASSERT_EQUAL_UINT32(2U, recorder.size());

    recorder.rearm();

    TEST_ASSERT_FALSE(recorder.frozen());
    TEST_ASSERT_TRUE(recorder.empty());
    TEST_ASSERT_EQUAL_UINT32(0U, recorder.size());

    recorder.record(3000U, sample);

    TEST_ASSERT_EQUAL_UINT32(1U, recorder.size());

    FastFlightSample recorded{};

    TEST_ASSERT_TRUE(recorder.get(0, recorded));
    TEST_ASSERT_EQUAL_UINT32(3000U, recorded.timestampUs);
}

void test_clear_preserves_frozen_state()
{
    FlightRecorder<FastFlightSample, 4> recorder;

    FastFlightSample sample{};

    recorder.record(1000U, sample);
    recorder.freeze();

    TEST_ASSERT_TRUE(recorder.frozen());

    recorder.clear();

    TEST_ASSERT_TRUE(recorder.empty());
    TEST_ASSERT_TRUE(recorder.frozen());

    recorder.record(2000U, sample);

    TEST_ASSERT_TRUE(recorder.empty());
}

int main(int, char**)
{
    UNITY_BEGIN();

    RUN_TEST(test_starts_empty);
    RUN_TEST(test_records_samples_in_order);
    RUN_TEST(test_overwrites_oldest_sample);
    RUN_TEST(test_invalid_get_leaves_output_unchanged);
    RUN_TEST(test_clear);
    RUN_TEST(test_freeze_stops_recording);
    RUN_TEST(test_unfreeze_resumes_recording);
    RUN_TEST(test_clear_does_not_unfreeze);
    RUN_TEST(test_record_stores_timestamp);
    RUN_TEST(test_rearm_clears_and_unfreezes_recorder);
    RUN_TEST(test_clear_preserves_frozen_state);

    return UNITY_END();
}
