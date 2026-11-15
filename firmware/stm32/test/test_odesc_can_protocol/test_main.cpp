#include <unity.h>

#include "../../src/motor/OdescCanProtocol.h"

void test_make_can_id()
{
    // CANSimple:
    // CAN ID = (nodeId << 5) | commandId

    TEST_ASSERT_EQUAL_HEX32(
        0x0E,
        OdescCanProtocol::makeCanId(
            0,
            OdescCanProtocol::kSetInputTorque));

    TEST_ASSERT_EQUAL_HEX32(
        0x2E,
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetInputTorque));

    TEST_ASSERT_EQUAL_HEX32(
        0x47,
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetAxisRequestedState));
}

void test_make_set_input_torque_frame()
{
    const CanFrame frame =
        OdescCanProtocol::makeSetInputTorqueFrame(
            1,
            1.0f);

    TEST_ASSERT_EQUAL_HEX32(
        0x2E,
        frame.id);

    TEST_ASSERT_EQUAL(4, frame.length);

    TEST_ASSERT_EQUAL_HEX8(0x00, frame.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x80, frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x3F, frame.data[3]);
}

void test_make_set_axis_requested_state_frame()
{
    const CanFrame frame =
        OdescCanProtocol::makeSetAxisRequestedStateFrame(
            2,
            OdescCanProtocol::kAxisStateClosedLoopControl);

    TEST_ASSERT_EQUAL_HEX32(
        0x47,
        frame.id);

    TEST_ASSERT_EQUAL(4, frame.length);

    TEST_ASSERT_EQUAL_HEX8(0x08, frame.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, frame.data[3]);
}

void test_parse_heartbeat()
{
    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kHeartbeat),
        .data = {
            0x78, 0x56, 0x34, 0x12,
            0x08, 0x00, 0x00, 0x00
        },
        .length = 8
    };

    OdescCanProtocol::Heartbeat heartbeat{};

    const bool parsed =
        OdescCanProtocol::parseHeartbeat(
            frame,
            2,
            heartbeat);

    TEST_ASSERT_TRUE(parsed);

    TEST_ASSERT_EQUAL_HEX32(
        0x12345678,
        heartbeat.axisError);

    TEST_ASSERT_EQUAL(
        OdescCanProtocol::kAxisStateClosedLoopControl,
        heartbeat.axisState);
}

void test_parse_heartbeat_rejects_wrong_node()
{
    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            3,
            OdescCanProtocol::kHeartbeat),
        .data = {
            0x00, 0x00, 0x00, 0x00,
            0x08, 0x00, 0x00, 0x00
        },
        .length = 8
    };

    OdescCanProtocol::Heartbeat heartbeat{};

    const bool parsed =
        OdescCanProtocol::parseHeartbeat(
            frame,
            2,
            heartbeat);

    TEST_ASSERT_FALSE(parsed);
}

void test_parse_heartbeat_rejects_short_frame()
{
    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kHeartbeat),
        .data = {
            0x00, 0x00, 0x00, 0x00
        },
        .length = 4
    };

    OdescCanProtocol::Heartbeat heartbeat{};

    const bool parsed =
        OdescCanProtocol::parseHeartbeat(
            frame,
            2,
            heartbeat);

    TEST_ASSERT_FALSE(parsed);
}

void test_parse_encoder_estimates()
{
    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            3,
            OdescCanProtocol::kGetEncoderEstimates),
        .data = {},
        .length = 8
    };

    OdescCanProtocol::writeFloat32LE(
        frame.data,
        1.5f);

    OdescCanProtocol::writeFloat32LE(
        frame.data + 4,
        -2.25f);

    OdescCanProtocol::EncoderEstimates estimates{};

    const bool result =
        OdescCanProtocol::parseEncoderEstimates(
            frame,
            3,
            estimates);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        1.5f,
        estimates.position);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        -2.25f,
        estimates.velocity);
}

void test_parse_encoder_estimates_rejects_wrong_can_id()
{
    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            4,
            OdescCanProtocol::kGetEncoderEstimates),
        .data = {},
        .length = 8
    };

    OdescCanProtocol::EncoderEstimates estimates{};

    const bool result =
        OdescCanProtocol::parseEncoderEstimates(
            frame,
            3,
            estimates);

    TEST_ASSERT_FALSE(result);
}

void test_parse_encoder_estimates_rejects_short_payload()
{
    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            3,
            OdescCanProtocol::kGetEncoderEstimates),
        .data = {},
        .length = 4
    };

    OdescCanProtocol::EncoderEstimates estimates{};

    const bool result =
        OdescCanProtocol::parseEncoderEstimates(
            frame,
            3,
            estimates);

    TEST_ASSERT_FALSE(result);
}

void test_parse_bus_voltage_current()
{
    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kGetBusVoltageCurrent),
        .data = {},
        .length = 8
    };

    OdescCanProtocol::writeFloat32LE(
        frame.data,
        24.5f);

    OdescCanProtocol::writeFloat32LE(
        frame.data + 4,
        3.25f);

    OdescCanProtocol::BusVoltageCurrent bus{};

    TEST_ASSERT_TRUE(
        OdescCanProtocol::parseBusVoltageCurrent(
            frame,
            1,
            bus));

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        24.5f,
        bus.voltage);

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        3.25f,
        bus.current);
}

void test_parse_bus_voltage_current_rejects_wrong_node()
{
    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kGetBusVoltageCurrent),
        .data = {},
        .length = 8
    };

    OdescCanProtocol::BusVoltageCurrent bus{};

    TEST_ASSERT_FALSE(
        OdescCanProtocol::parseBusVoltageCurrent(
            frame,
            1,
            bus));
}

void test_parse_bus_voltage_current_rejects_short_frame()
{
    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kGetBusVoltageCurrent),
        .data = {},
        .length = 7
    };

    OdescCanProtocol::BusVoltageCurrent bus{};

    TEST_ASSERT_FALSE(
        OdescCanProtocol::parseBusVoltageCurrent(
            frame,
            1,
            bus));
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_make_can_id);
    RUN_TEST(test_make_set_input_torque_frame);
    RUN_TEST(test_make_set_axis_requested_state_frame);
    RUN_TEST(test_parse_heartbeat);
    RUN_TEST(test_parse_heartbeat_rejects_wrong_node);
    RUN_TEST(test_parse_heartbeat_rejects_short_frame);
    RUN_TEST(test_parse_encoder_estimates);
    RUN_TEST(test_parse_encoder_estimates_rejects_wrong_can_id);
    RUN_TEST(test_parse_encoder_estimates_rejects_short_payload);
    RUN_TEST(test_parse_bus_voltage_current);
    RUN_TEST(test_parse_bus_voltage_current_rejects_wrong_node);
    RUN_TEST(test_parse_bus_voltage_current_rejects_short_frame);

    return UNITY_END();
}
