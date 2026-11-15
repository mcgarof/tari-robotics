#include <unity.h>

#include "control/MotionCommandProtocol.h"
#include "control/MotionCommandReceiver.h"

#include <cstddef>
#include <cstdint>
#include <limits>

namespace
{

using MotionCommandProtocol::Packet;

MotionCommandReceiver makeReceiver()
{
    return MotionCommandReceiver({
        .maxVelocity = 2.0f,
        .maxSteeringRate = 1.5f,
    });
}

Packet makePacket(
    uint16_t sequence,
    float velocity = 1.0f,
    float steering = -0.5f,
    uint32_t sessionId = 0x12345678)
{
    return {
        .sessionId = sessionId,
        .sequence = sequence,
        .velocity = velocity,
        .steering = steering,
        .flags = 0,
    };
}

bool feed(
    MotionCommandReceiver& receiver,
    const uint8_t* data,
    std::size_t size,
    Packet& output)
{
    bool received = false;

    for (std::size_t i = 0; i < size; ++i)
    {
        if (receiver.pushByte(data[i], output))
        {
            received = true;
        }
    }

    return received;
}

bool feedPacket(
    MotionCommandReceiver& receiver,
    const Packet& input,
    Packet& output)
{
    uint8_t encoded[MotionCommandProtocol::kPacketSize];

    if (!MotionCommandProtocol::encode(
            input,
            encoded,
            sizeof(encoded)))
    {
        return false;
    }

    return feed(receiver, encoded, sizeof(encoded), output);
}

void test_valid_packet_is_accepted()
{
    auto receiver = makeReceiver();
    Packet output{};

    TEST_ASSERT_TRUE(
        feedPacket(receiver, makePacket(10), output));

    TEST_ASSERT_EQUAL_UINT16(10, output.sequence);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, output.velocity);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, output.steering);
}

void test_duplicate_packet_is_rejected()
{
    auto receiver = makeReceiver();
    Packet output{};

    TEST_ASSERT_TRUE(
        feedPacket(receiver, makePacket(20), output));

    TEST_ASSERT_FALSE(
        feedPacket(receiver, makePacket(20), output));
}

void test_old_packet_is_rejected()
{
    auto receiver = makeReceiver();
    Packet output{};

    TEST_ASSERT_TRUE(
        feedPacket(receiver, makePacket(30), output));

    TEST_ASSERT_FALSE(
        feedPacket(receiver, makePacket(29), output));
}

void test_new_session_is_accepted()
{
    auto receiver = makeReceiver();
    Packet output{};

    TEST_ASSERT_TRUE(
        feedPacket(
            receiver,
            makePacket(100, 1.0f, 0.0f, 0x11111111),
            output));

    TEST_ASSERT_TRUE(
        feedPacket(
            receiver,
            makePacket(1, 1.0f, 0.0f, 0x22222222),
            output));

    TEST_ASSERT_EQUAL_UINT32(0x22222222, output.sessionId);
    TEST_ASSERT_EQUAL_UINT16(1, output.sequence);
}

void test_out_of_range_command_is_rejected()
{
    auto receiver = makeReceiver();
    Packet output{};

    TEST_ASSERT_FALSE(
        feedPacket(
            receiver,
            makePacket(40, 2.01f, 0.0f),
            output));

    TEST_ASSERT_FALSE(
        feedPacket(
            receiver,
            makePacket(41, 0.0f, 1.51f),
            output));
}

void test_nan_command_is_rejected()
{
    auto receiver = makeReceiver();
    Packet output{};

    const float nan =
        std::numeric_limits<float>::quiet_NaN();

    TEST_ASSERT_FALSE(
        feedPacket(
            receiver,
            makePacket(50, nan, 0.0f),
            output));
}

void test_corrupted_packet_is_rejected()
{
    auto receiver = makeReceiver();

    uint8_t encoded[MotionCommandProtocol::kPacketSize];

    MotionCommandProtocol::encode(
        makePacket(60),
        encoded,
        sizeof(encoded));

    encoded[10] ^= 0x01;

    Packet output{};

    TEST_ASSERT_FALSE(
        feed(receiver, encoded, sizeof(encoded), output));
}

void test_rejected_packet_does_not_modify_output()
{
    auto receiver = makeReceiver();

    Packet output{
        .sessionId = 0xAAAAAAAA,
        .sequence = 123,
        .velocity = 0.25f,
        .steering = 0.5f,
        .flags = 0x55,
    };

    TEST_ASSERT_FALSE(
        feedPacket(
            receiver,
            makePacket(70, 100.0f, 0.0f),
            output));

    TEST_ASSERT_EQUAL_UINT32(0xAAAAAAAA, output.sessionId);
    TEST_ASSERT_EQUAL_UINT16(123, output.sequence);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, output.velocity);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, output.steering);
    TEST_ASSERT_EQUAL_UINT8(0x55, output.flags);
}

void test_reset_allows_sequence_again()
{
    auto receiver = makeReceiver();
    Packet output{};

    TEST_ASSERT_TRUE(
        feedPacket(receiver, makePacket(80), output));

    TEST_ASSERT_FALSE(
        feedPacket(receiver, makePacket(80), output));

    receiver.reset();

    TEST_ASSERT_TRUE(
        feedPacket(receiver, makePacket(80), output));
}

}

int main(int, char**)
{
    UNITY_BEGIN();

    RUN_TEST(test_valid_packet_is_accepted);
    RUN_TEST(test_duplicate_packet_is_rejected);
    RUN_TEST(test_old_packet_is_rejected);
    RUN_TEST(test_new_session_is_accepted);
    RUN_TEST(test_out_of_range_command_is_rejected);
    RUN_TEST(test_nan_command_is_rejected);
    RUN_TEST(test_corrupted_packet_is_rejected);
    RUN_TEST(test_rejected_packet_does_not_modify_output);
    RUN_TEST(test_reset_allows_sequence_again);

    return UNITY_END();
}
