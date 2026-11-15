#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <cstring>

#include <unity.h>

#include "observability/Event.h"
#include "observability/EventDumpPublisher.h"
#include "observability/EventRecorder.h"
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

void test_second_dump_publishes_only_new_events()
{
    EventRecorder<4> recorder;

    recorder.record(
        1000U,
        EventId::IMU_FAULT,
        EventSeverity::ERROR);

    recorder.record(
        2000U,
        EventId::COMMAND_TIMEOUT,
        EventSeverity::WARNING);

    EventDumpPublisher<4> dump{
        serializeEvent};

    FakeTransport transport{5};

    TEST_ASSERT_TRUE(dump.start());

    while (dump.active())
    {
        dump.update(recorder, transport);
    }

    TEST_ASSERT_EQUAL_STRING(
        "E,1,1000,0,1,2\n"
        "E,1,2000,1,3,1\n",
        transport.output().c_str());

    recorder.record(
        3000U,
        EventId::MOTOR_FAULT,
        EventSeverity::ERROR);

    TEST_ASSERT_TRUE(dump.start());

    while (dump.active())
    {
        dump.update(recorder, transport);
    }

    TEST_ASSERT_EQUAL_STRING(
        "E,1,1000,0,1,2\n"
        "E,1,2000,1,3,1\n"
        "E,1,3000,2,5,2\n",
        transport.output().c_str());
}

void test_serialize_events_lost()
{
    char buffer[128]{};

    const std::size_t length =
        serializeEventsLost(
            5U,
            9U,
            buffer,
            sizeof(buffer));

    TEST_ASSERT_GREATER_THAN(0, length);

    TEST_ASSERT_EQUAL_STRING(
        "L,1,5,9\n",
        buffer);

    TEST_ASSERT_EQUAL_UINT(
        std::strlen(buffer),
        length);
}

void test_next_sequence_tracks_published_events()
{
    EventPublisher publisher;

    TEST_ASSERT_EQUAL_UINT32(
        0U,
        publisher.nextSequence());

    Event event{};
    event.sequence = 7U;

    publisher.markPublished(event);

    TEST_ASSERT_EQUAL_UINT32(
        8U,
        publisher.nextSequence());
}

void test_events_lost_is_reported_before_oldest_available_event()
{
    EventRecorder<3> recorder;

    recorder.record(
        1000U,
        EventId::IMU_FAULT,
        EventSeverity::ERROR);

    recorder.record(
        2000U,
        EventId::COMMAND_TIMEOUT,
        EventSeverity::WARNING);

    recorder.record(
        3000U,
        EventId::MOTOR_FAULT,
        EventSeverity::ERROR);

    recorder.record(
        4000U,
        EventId::FAULT_ENTERED,
        EventSeverity::CRITICAL);

    EventDumpPublisher<3> dump{
        serializeEvent};

    FakeTransport transport{5};

    TEST_ASSERT_TRUE(dump.start());

    while (dump.active())
    {
        dump.update(recorder, transport);
    }

    TEST_ASSERT_EQUAL_STRING(
        "L,1,0,1\n"
        "E,1,2000,1,3,1\n"
        "E,1,3000,2,5,2\n"
        "E,1,4000,3,7,3\n",
        transport.output().c_str());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_second_dump_publishes_only_new_events);
    RUN_TEST(test_serialize_events_lost);
    RUN_TEST(test_next_sequence_tracks_published_events);
    RUN_TEST(test_events_lost_is_reported_before_oldest_available_event);

    return UNITY_END();
}
