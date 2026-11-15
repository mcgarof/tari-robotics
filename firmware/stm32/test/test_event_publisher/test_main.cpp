#include <unity.h>

#include "observability/EventPublisher.h"
#include "observability/EventRecorder.h"

void setUp() {}
void tearDown() {}

void test_next_returns_false_when_recorder_is_empty()
{
    EventRecorder<4> recorder;
    EventPublisher publisher;

    Event event{};

    TEST_ASSERT_EQUAL(
        EventPublishStatus::NO_EVENT,
        publisher.next(recorder, event));
}

void test_next_returns_first_unpublished_event()
{
    EventRecorder<4> recorder;
    EventPublisher publisher;

    recorder.record(
        100,
        EventId::IMU_FAULT,
        EventSeverity::ERROR);

    Event event{};

    TEST_ASSERT_EQUAL(
        EventPublishStatus::EVENT_AVAILABLE,
        publisher.next(recorder, event));

    TEST_ASSERT_EQUAL_UINT32(0, event.sequence);
    TEST_ASSERT_EQUAL_UINT32(100, event.timestampUs);
    TEST_ASSERT_EQUAL(EventId::IMU_FAULT, event.id);
    TEST_ASSERT_EQUAL(EventSeverity::ERROR, event.severity);
}

void test_next_reports_lost_events_when_expected_sequence_was_overwritten()
{
    EventRecorder<3> recorder;
    EventPublisher publisher;

    // seq 0, 1 and 2
    recorder.record(100, EventId::IMU_FAULT, EventSeverity::ERROR);
    recorder.record(200, EventId::COMMAND_TIMEOUT, EventSeverity::WARNING);
    recorder.record(300, EventId::TILT_EXCEEDED, EventSeverity::ERROR);

    // seq 3 overwrites seq 0
    recorder.record(400, EventId::CONTROL_OVERRUN, EventSeverity::WARNING);

    Event event{};

    TEST_ASSERT_EQUAL(
        EventPublishStatus::EVENTS_LOST,
        publisher.next(recorder, event));

    // The oldest event still available is seq 1.
    TEST_ASSERT_EQUAL_UINT32(1, event.sequence);
    TEST_ASSERT_EQUAL_UINT32(200, event.timestampUs);
}

void test_sequence_before_handles_normal_ordering()
{
    TEST_ASSERT_TRUE(sequenceBefore(5, 8));
    TEST_ASSERT_FALSE(sequenceBefore(8, 5));
    TEST_ASSERT_FALSE(sequenceBefore(5, 5));
}

void test_sequence_before_handles_uint32_wrap()
{
    TEST_ASSERT_TRUE(sequenceBefore(0xFFFFFFFFu, 0u));
    TEST_ASSERT_TRUE(sequenceBefore(0u, 1u));

    TEST_ASSERT_FALSE(sequenceBefore(0u, 0xFFFFFFFFu));
    TEST_ASSERT_FALSE(sequenceBefore(1u, 0u));
}

void test_mark_published_advances_to_next_event()
{
    EventRecorder<4> recorder;
    EventPublisher publisher;

    recorder.record(100, EventId::IMU_FAULT, EventSeverity::ERROR);
    recorder.record(200, EventId::COMMAND_TIMEOUT, EventSeverity::WARNING);

    Event event{};

    TEST_ASSERT_EQUAL(
        EventPublishStatus::EVENT_AVAILABLE,
        publisher.next(recorder, event));

    TEST_ASSERT_EQUAL_UINT32(0, event.sequence);

    publisher.markPublished(event);

    TEST_ASSERT_EQUAL(
        EventPublishStatus::EVENT_AVAILABLE,
        publisher.next(recorder, event));

    TEST_ASSERT_EQUAL_UINT32(1, event.sequence);
    TEST_ASSERT_EQUAL_UINT32(200, event.timestampUs);
}

void test_next_does_not_advance_without_mark_published()
{
    EventRecorder<4> recorder;
    EventPublisher publisher;

    recorder.record(
        100,
        EventId::IMU_FAULT,
        EventSeverity::ERROR);

    Event first{};
    Event second{};

    TEST_ASSERT_EQUAL(
        EventPublishStatus::EVENT_AVAILABLE,
        publisher.next(recorder, first));

    TEST_ASSERT_EQUAL(
        EventPublishStatus::EVENT_AVAILABLE,
        publisher.next(recorder, second));

    TEST_ASSERT_EQUAL_UINT32(0, first.sequence);
    TEST_ASSERT_EQUAL_UINT32(0, second.sequence);

    TEST_ASSERT_EQUAL_UINT32(
        first.timestampUs,
        second.timestampUs);
}

void test_mark_published_recovers_after_events_lost()
{
    EventRecorder<3> recorder;
    EventPublisher publisher;

    // seq 0, 1, 2
    recorder.record(100, EventId::IMU_FAULT, EventSeverity::ERROR);
    recorder.record(200, EventId::COMMAND_TIMEOUT, EventSeverity::WARNING);
    recorder.record(300, EventId::TILT_EXCEEDED, EventSeverity::ERROR);

    // seq 3 overwrites seq 0
    recorder.record(400, EventId::CONTROL_OVERRUN, EventSeverity::WARNING);

    Event event{};

    TEST_ASSERT_EQUAL(
        EventPublishStatus::EVENTS_LOST,
        publisher.next(recorder, event));

    // First event still available is seq 1.
    TEST_ASSERT_EQUAL_UINT32(1, event.sequence);

    // Assume seq 1 was successfully transmitted.
    publisher.markPublished(event);

    TEST_ASSERT_EQUAL(
        EventPublishStatus::EVENT_AVAILABLE,
        publisher.next(recorder, event));

    TEST_ASSERT_EQUAL_UINT32(2, event.sequence);
    TEST_ASSERT_EQUAL_UINT32(300, event.timestampUs);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_next_returns_false_when_recorder_is_empty);
    RUN_TEST(test_next_returns_first_unpublished_event);
    RUN_TEST(test_next_reports_lost_events_when_expected_sequence_was_overwritten);
    RUN_TEST(test_sequence_before_handles_normal_ordering);
    RUN_TEST(test_sequence_before_handles_uint32_wrap);
    RUN_TEST(test_mark_published_advances_to_next_event);
    RUN_TEST(test_next_does_not_advance_without_mark_published);
    RUN_TEST(test_mark_published_recovers_after_events_lost);

    return UNITY_END();
}
