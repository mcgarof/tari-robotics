#include <unity.h>

#include "observability/EventRecorder.h"

void setUp() {}
void tearDown() {}

void test_starts_empty() {
    EventRecorder<4> recorder;

    TEST_ASSERT_TRUE(recorder.empty());
    TEST_ASSERT_EQUAL_UINT32(0, recorder.size());
    TEST_ASSERT_EQUAL_UINT32(4, recorder.capacity());
}

void test_records_events_in_order() {
    EventRecorder<4> recorder;
    Event event;

    recorder.record(100, EventId::IMU_FAULT, EventSeverity::ERROR);
    recorder.record(200, EventId::COMMAND_TIMEOUT, EventSeverity::WARNING);

    TEST_ASSERT_EQUAL_UINT32(2, recorder.size());
    TEST_ASSERT_FALSE(recorder.empty());

    TEST_ASSERT_TRUE(recorder.get(0, event));
    TEST_ASSERT_EQUAL_UINT32(100, event.timestampUs);
    TEST_ASSERT_EQUAL(EventId::IMU_FAULT, event.id);
    TEST_ASSERT_EQUAL(EventSeverity::ERROR, event.severity);

    TEST_ASSERT_TRUE(recorder.get(1, event));
    TEST_ASSERT_EQUAL_UINT32(200, event.timestampUs);
    TEST_ASSERT_EQUAL(EventId::COMMAND_TIMEOUT, event.id);
    TEST_ASSERT_EQUAL(EventSeverity::WARNING, event.severity);
}

void test_overwrites_oldest_event_when_full() {
    EventRecorder<3> recorder;
    Event event;

    recorder.record(100, EventId::IMU_FAULT, EventSeverity::ERROR);
    recorder.record(200, EventId::COMMAND_TIMEOUT, EventSeverity::WARNING);
    recorder.record(300, EventId::TILT_EXCEEDED, EventSeverity::CRITICAL);
    recorder.record(400, EventId::SAFE_STOP_ENTERED, EventSeverity::ERROR);

    TEST_ASSERT_EQUAL_UINT32(3, recorder.size());

    TEST_ASSERT_TRUE(recorder.get(0, event));
    TEST_ASSERT_EQUAL_UINT32(200, event.timestampUs);
    TEST_ASSERT_TRUE(recorder.get(1, event));
    TEST_ASSERT_EQUAL_UINT32(300, event.timestampUs);
    TEST_ASSERT_TRUE(recorder.get(2, event));
    TEST_ASSERT_EQUAL_UINT32(400, event.timestampUs);

    TEST_ASSERT_TRUE(recorder.get(0, event));
    TEST_ASSERT_EQUAL(EventId::COMMAND_TIMEOUT, event.id);
    TEST_ASSERT_TRUE(recorder.get(1, event));
    TEST_ASSERT_EQUAL(EventId::TILT_EXCEEDED, event.id);
    TEST_ASSERT_TRUE(recorder.get(2, event));
    TEST_ASSERT_EQUAL(EventId::SAFE_STOP_ENTERED, event.id);
}

void test_clear_removes_all_events() {
    EventRecorder<4> recorder;
    Event event;

    recorder.record(100, EventId::CONTROL_OVERRUN, EventSeverity::WARNING);
    recorder.record(200, EventId::FAULT_ENTERED, EventSeverity::CRITICAL);

    recorder.clear();

    TEST_ASSERT_TRUE(recorder.empty());
    TEST_ASSERT_EQUAL_UINT32(0, recorder.size());

    recorder.record(300, EventId::PARAM_CHANGE, EventSeverity::INFO);

    TEST_ASSERT_EQUAL_UINT32(1, recorder.size());
    TEST_ASSERT_TRUE(recorder.get(0, event));
    TEST_ASSERT_EQUAL_UINT32(300, event.timestampUs);
    TEST_ASSERT_EQUAL(EventId::PARAM_CHANGE, event.id);
}

void test_get_rejects_invalid_index() {
    EventRecorder<2> recorder;

    recorder.record(100, EventId::IMU_FAULT, EventSeverity::ERROR);

    Event event{
        999,
        5,
        EventId::PARAM_CHANGE,
        EventSeverity::INFO
    };

    TEST_ASSERT_FALSE(recorder.get(1, event));

    // A failed read must not modify the output event.
    TEST_ASSERT_EQUAL_UINT32(999, event.timestampUs);
    TEST_ASSERT_EQUAL_UINT32(5, event.sequence);
    TEST_ASSERT_EQUAL(EventId::PARAM_CHANGE, event.id);
    TEST_ASSERT_EQUAL(EventSeverity::INFO, event.severity);
}

void test_sequence_increments_for_each_recorded_event()
{
    EventRecorder<4> recorder;

    recorder.record(100, EventId::IMU_FAULT, EventSeverity::ERROR);
    recorder.record(200, EventId::COMMAND_TIMEOUT, EventSeverity::WARNING);
    recorder.record(300, EventId::TILT_EXCEEDED, EventSeverity::ERROR);

    Event event{};

    TEST_ASSERT_TRUE(recorder.get(0, event));
    TEST_ASSERT_EQUAL_UINT32(0, event.sequence);

    TEST_ASSERT_TRUE(recorder.get(1, event));
    TEST_ASSERT_EQUAL_UINT32(1, event.sequence);

    TEST_ASSERT_TRUE(recorder.get(2, event));
    TEST_ASSERT_EQUAL_UINT32(2, event.sequence);
}

void test_clear_does_not_reset_sequence()
{
    EventRecorder<4> recorder;

    recorder.record(100, EventId::IMU_FAULT, EventSeverity::ERROR);
    recorder.record(200, EventId::COMMAND_TIMEOUT, EventSeverity::WARNING);

    recorder.clear();

    recorder.record(300, EventId::TILT_EXCEEDED, EventSeverity::ERROR);

    Event event{};

    TEST_ASSERT_TRUE(recorder.get(0, event));
    TEST_ASSERT_EQUAL_UINT32(2, event.sequence);
}

void test_sequence_is_preserved_when_old_events_are_overwritten()
{
    EventRecorder<3> recorder;

    recorder.record(100, EventId::IMU_FAULT, EventSeverity::ERROR);
    recorder.record(200, EventId::COMMAND_TIMEOUT, EventSeverity::WARNING);
    recorder.record(300, EventId::TILT_EXCEEDED, EventSeverity::ERROR);
    recorder.record(400, EventId::CONTROL_OVERRUN, EventSeverity::WARNING);
    recorder.record(500, EventId::SAFE_STOP_ENTERED, EventSeverity::ERROR);

    TEST_ASSERT_EQUAL_UINT32(3, recorder.size());

    Event event{};

    TEST_ASSERT_TRUE(recorder.get(0, event));
    TEST_ASSERT_EQUAL_UINT32(2, event.sequence);
    TEST_ASSERT_EQUAL_UINT32(300, event.timestampUs);

    TEST_ASSERT_TRUE(recorder.get(1, event));
    TEST_ASSERT_EQUAL_UINT32(3, event.sequence);
    TEST_ASSERT_EQUAL_UINT32(400, event.timestampUs);

    TEST_ASSERT_TRUE(recorder.get(2, event));
    TEST_ASSERT_EQUAL_UINT32(4, event.sequence);
    TEST_ASSERT_EQUAL_UINT32(500, event.timestampUs);
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_starts_empty);
    RUN_TEST(test_records_events_in_order);
    RUN_TEST(test_overwrites_oldest_event_when_full);
    RUN_TEST(test_clear_removes_all_events);
    RUN_TEST(test_get_rejects_invalid_index);
    RUN_TEST(test_sequence_increments_for_each_recorded_event);
    RUN_TEST(test_clear_does_not_reset_sequence);
    RUN_TEST(test_sequence_is_preserved_when_old_events_are_overwritten);

    return UNITY_END();
}
