#include <unity.h>

#include "control/MotionCommandProtocol.h"
#include "control/MotionCommandStreamParser.h"

#include <cstddef>
#include <cstdint>

namespace
{

using MotionCommandProtocol::Packet;

Packet makePacket(uint16_t sequence)
{
    return {
        .sessionId = 0x12345678,
        .sequence = sequence,
        .velocity = 1.0f,
        .steering = -0.5f,
        .flags = 0,
    };
}

bool feedPacket(
    MotionCommandStreamParser& parser,
    const uint8_t* data,
    std::size_t size,
    Packet& packet)
{
    bool received = false;

    for (std::size_t i = 0; i < size; ++i)
    {
        if (parser.pushByte(data[i], packet))
        {
            received = true;
        }
    }

    return received;
}

void test_valid_packet_is_received()
{
    MotionCommandStreamParser parser;

    const Packet input = makePacket(10);

    uint8_t encoded[MotionCommandProtocol::kPacketSize];
    TEST_ASSERT_TRUE(
        MotionCommandProtocol::encode(
            input,
            encoded,
            sizeof(encoded)));

    Packet output{};

    TEST_ASSERT_TRUE(
        feedPacket(parser, encoded, sizeof(encoded), output));

    TEST_ASSERT_EQUAL_UINT32(input.sessionId, output.sessionId);
    TEST_ASSERT_EQUAL_UINT16(input.sequence, output.sequence);
    TEST_ASSERT_EQUAL_FLOAT(input.velocity, output.velocity);
    TEST_ASSERT_EQUAL_FLOAT(input.steering, output.steering);
}

void test_leading_garbage_is_ignored()
{
    MotionCommandStreamParser parser;

    const uint8_t garbage[] = {
        0x00, 0x11, 0xA5, 0xFF, 0x42
    };

    Packet output{};

    TEST_ASSERT_FALSE(
        feedPacket(parser, garbage, sizeof(garbage), output));

    const Packet input = makePacket(20);

    uint8_t encoded[MotionCommandProtocol::kPacketSize];
    MotionCommandProtocol::encode(
        input,
        encoded,
        sizeof(encoded));

    TEST_ASSERT_TRUE(
        feedPacket(parser, encoded, sizeof(encoded), output));

    TEST_ASSERT_EQUAL_UINT16(20, output.sequence);
}

void test_packet_can_arrive_in_parts()
{
    MotionCommandStreamParser parser;

    const Packet input = makePacket(30);

    uint8_t encoded[MotionCommandProtocol::kPacketSize];
    MotionCommandProtocol::encode(
        input,
        encoded,
        sizeof(encoded));

    Packet output{};

    TEST_ASSERT_FALSE(
        feedPacket(parser, encoded, 7, output));

    TEST_ASSERT_TRUE(
        feedPacket(
            parser,
            encoded + 7,
            sizeof(encoded) - 7,
            output));

    TEST_ASSERT_EQUAL_UINT16(30, output.sequence);
}

void test_corrupted_packet_is_rejected()
{
    MotionCommandStreamParser parser;

    const Packet input = makePacket(40);

    uint8_t encoded[MotionCommandProtocol::kPacketSize];
    MotionCommandProtocol::encode(
        input,
        encoded,
        sizeof(encoded));

    encoded[10] ^= 0x01;

    Packet output{};

    TEST_ASSERT_FALSE(
        feedPacket(parser, encoded, sizeof(encoded), output));
}

void test_valid_packet_after_corrupted_packet_is_received()
{
    MotionCommandStreamParser parser;

    uint8_t corrupted[MotionCommandProtocol::kPacketSize];
    uint8_t valid[MotionCommandProtocol::kPacketSize];

    MotionCommandProtocol::encode(
        makePacket(50),
        corrupted,
        sizeof(corrupted));

    MotionCommandProtocol::encode(
        makePacket(51),
        valid,
        sizeof(valid));

    corrupted[12] ^= 0x80;

    Packet output{};

    TEST_ASSERT_FALSE(
        feedPacket(parser, corrupted, sizeof(corrupted), output));

    TEST_ASSERT_TRUE(
        feedPacket(parser, valid, sizeof(valid), output));

    TEST_ASSERT_EQUAL_UINT16(51, output.sequence);
}

void test_two_consecutive_packets_are_received()
{
    MotionCommandStreamParser parser;

    uint8_t first[MotionCommandProtocol::kPacketSize];
    uint8_t second[MotionCommandProtocol::kPacketSize];

    MotionCommandProtocol::encode(
        makePacket(60),
        first,
        sizeof(first));

    MotionCommandProtocol::encode(
        makePacket(61),
        second,
        sizeof(second));

    Packet output{};

    TEST_ASSERT_TRUE(
        feedPacket(parser, first, sizeof(first), output));

    TEST_ASSERT_EQUAL_UINT16(60, output.sequence);

    TEST_ASSERT_TRUE(
        feedPacket(parser, second, sizeof(second), output));

    TEST_ASSERT_EQUAL_UINT16(61, output.sequence);
}

void test_repeated_first_sync_byte_is_handled()
{
    MotionCommandStreamParser parser;

    const uint8_t prefix[] = {
        0x5A
    };

    Packet output{};

    TEST_ASSERT_FALSE(
        feedPacket(parser, prefix, sizeof(prefix), output));

    uint8_t encoded[MotionCommandProtocol::kPacketSize];

    MotionCommandProtocol::encode(
        makePacket(70),
        encoded,
        sizeof(encoded));

    TEST_ASSERT_TRUE(
        feedPacket(parser, encoded, sizeof(encoded), output));

    TEST_ASSERT_EQUAL_UINT16(70, output.sequence);
}

void test_reset_discards_partial_packet()
{
    MotionCommandStreamParser parser;

    uint8_t encoded[MotionCommandProtocol::kPacketSize];

    MotionCommandProtocol::encode(
        makePacket(80),
        encoded,
        sizeof(encoded));

    Packet output{};

    TEST_ASSERT_FALSE(
        feedPacket(parser, encoded, 10, output));

    parser.reset();

    TEST_ASSERT_TRUE(
        feedPacket(parser, encoded, sizeof(encoded), output));

    TEST_ASSERT_EQUAL_UINT16(80, output.sequence);
}

void test_recovers_after_lost_byte()
{
    MotionCommandStreamParser parser;

    uint8_t first[MotionCommandProtocol::kPacketSize];
    uint8_t second[MotionCommandProtocol::kPacketSize];

    MotionCommandProtocol::encode(
        makePacket(90),
        first,
        sizeof(first));

    MotionCommandProtocol::encode(
        makePacket(91),
        second,
        sizeof(second));

    Packet output{};

    // Simulate one lost byte inside the first packet.
    constexpr std::size_t lostIndex = 10;

    for (std::size_t i = 0; i < sizeof(first); ++i)
    {
        if (i != lostIndex)
        {
            TEST_ASSERT_FALSE(
                parser.pushByte(first[i], output));
        }
    }

    bool received = false;

    for (std::size_t i = 0; i < sizeof(second); ++i)
    {
        if (parser.pushByte(second[i], output))
        {
            received = true;
        }
    }

    TEST_ASSERT_TRUE(received);
    TEST_ASSERT_EQUAL_UINT16(91, output.sequence);
}

}

int main(int, char**)
{
    UNITY_BEGIN();

    RUN_TEST(test_valid_packet_is_received);
    RUN_TEST(test_leading_garbage_is_ignored);
    RUN_TEST(test_packet_can_arrive_in_parts);
    RUN_TEST(test_corrupted_packet_is_rejected);
    RUN_TEST(test_valid_packet_after_corrupted_packet_is_received);
    RUN_TEST(test_two_consecutive_packets_are_received);
    RUN_TEST(test_repeated_first_sync_byte_is_handled);
    RUN_TEST(test_reset_discards_partial_packet);
    RUN_TEST(test_recovers_after_lost_byte);

    return UNITY_END();
}
