#include <unity.h>

#include "../../src/control/MotionCommandProtocol.h"

using namespace MotionCommandProtocol;

void test_encode_decode_round_trip()
{
    Packet input{
        .sessionId = 0x12345678,
        .sequence = 42,
        .velocity = 1.25f,
        .steering = -0.5f,
        .flags = 0x03
    };

    uint8_t buffer[kPacketSize]{};

    TEST_ASSERT_TRUE(
        encode(input, buffer, sizeof(buffer)));

    Packet output{};

    TEST_ASSERT_TRUE(
        decode(buffer, sizeof(buffer), output));

    TEST_ASSERT_EQUAL_UINT32(
        input.sessionId,
        output.sessionId);

    TEST_ASSERT_EQUAL_UINT16(
        input.sequence,
        output.sequence);

    TEST_ASSERT_FLOAT_WITHIN(
        0.00001f,
        input.velocity,
        output.velocity);

    TEST_ASSERT_FLOAT_WITHIN(
        0.00001f,
        input.steering,
        output.steering);

    TEST_ASSERT_EQUAL_UINT8(
        input.flags,
        output.flags);
}

void test_decode_rejects_corrupted_packet()
{
    Packet packet{
        .sessionId = 123,
        .sequence = 10,
        .velocity = 1.0f,
        .steering = 0.25f,
        .flags = 0
    };

    uint8_t buffer[kPacketSize]{};

    TEST_ASSERT_TRUE(
        encode(packet, buffer, sizeof(buffer)));

    // Corrupt one byte inside velocity.
    buffer[10] ^= 0x01;

    Packet decoded{};

    TEST_ASSERT_FALSE(
        decode(buffer, sizeof(buffer), decoded));
}

void test_decode_rejects_wrong_sync()
{
    Packet packet{
        .sessionId = 123,
        .sequence = 10,
        .velocity = 1.0f,
        .steering = 0.25f,
        .flags = 0
    };

    uint8_t buffer[kPacketSize]{};

    TEST_ASSERT_TRUE(
        encode(packet, buffer, sizeof(buffer)));

    buffer[0] ^= 0x01;

    Packet decoded{};

    TEST_ASSERT_FALSE(
        decode(buffer, sizeof(buffer), decoded));
}

void test_decode_rejects_wrong_version()
{
    Packet packet{
        .sessionId = 123,
        .sequence = 10,
        .velocity = 1.0f,
        .steering = 0.25f,
        .flags = 0
    };

    uint8_t buffer[kPacketSize]{};

    TEST_ASSERT_TRUE(
        encode(packet, buffer, sizeof(buffer)));

    // Version is covered by the CRC, but decode()
    // checks the version before checking the CRC.
    buffer[2] = kVersion + 1;

    Packet decoded{};

    TEST_ASSERT_FALSE(
        decode(buffer, sizeof(buffer), decoded));
}

void test_encode_matches_golden_packet()
{
    Packet packet{
        .sessionId = 0x12345678,
        .sequence = 0x1234,
        .velocity = 1.0f,
        .steering = -0.5f,
        .flags = 0x03
    };

    uint8_t buffer[kPacketSize]{};

    TEST_ASSERT_TRUE(
        encode(packet, buffer, sizeof(buffer)));

    const uint8_t expected[kPacketSize]{
        0x5A, 0xA5,             // sync
        0x01,                   // version
        0x78, 0x56, 0x34, 0x12, // sessionId
        0x34, 0x12,             // sequence
        0x00, 0x00, 0x80, 0x3F, // velocity = 1.0f
        0x00, 0x00, 0x00, 0xBF, // steering = -0.5f
        0x03,                   // flags
        0x22, 0x6E              // CRC16 = 0x6E22
    };

    TEST_ASSERT_EQUAL_UINT8_ARRAY(
        expected,
        buffer,
        kPacketSize);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_encode_decode_round_trip);
    RUN_TEST(test_decode_rejects_corrupted_packet);
    RUN_TEST(test_decode_rejects_wrong_sync);
    RUN_TEST(test_decode_rejects_wrong_version);
    RUN_TEST(test_encode_matches_golden_packet);

    return UNITY_END();
}
