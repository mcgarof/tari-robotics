#include <unity.h>
#include <limits>

#include "../../src/motor/OdescMotorController.h"
#include "../../src/motor/OdescCanProtocol.h"

class FakeCanBus : public CanBus
{
public:
    bool send(const CanFrame& frame) override
    {
        if (frameCount < 8)
        {
            frames[frameCount++] = frame;
        }

        return true;
    }

    bool receive(CanFrame& frame) override
    {
        if (rxFrameIndex >= rxFrameCount)
        {
            return false;
        }

        frame = rxFrames[rxFrameIndex++];
        return true;
    }

    CanFrame frames[8]{};
    int frameCount = 0;

    CanFrame rxFrames[8]{};
    int rxFrameCount = 0;
    int rxFrameIndex = 0;
};

void test_initial_disabled_command_sends_nothing()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    ActuatorCommand command{
        .driveEnabled = false,
        .leftTorque = 10.0f,
        .rightTorque = -10.0f
    };

    motorController.apply(command);

    TEST_ASSERT_EQUAL(0, canBus.frameCount);
}

void test_enabling_drive_requests_closed_loop_and_sends_zero_torque()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    ActuatorCommand command{
        .driveEnabled = true,
        .leftTorque = 1.0f,
        .rightTorque = -1.0f
    };

    motorController.apply(command);

    TEST_ASSERT_EQUAL(4, canBus.frameCount);

    // Request CLOSED_LOOP_CONTROL on both axes.
    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetAxisRequestedState),
        canBus.frames[0].id);

    TEST_ASSERT_EQUAL_HEX8(
        0x08,
        canBus.frames[0].data[0]);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetAxisRequestedState),
        canBus.frames[1].id);

    TEST_ASSERT_EQUAL_HEX8(
        0x08,
        canBus.frames[1].data[0]);

    // CLOSED_LOOP_CONTROL has not been confirmed yet:
    // torque must remain zero.
    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[2].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[2].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[2].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[2].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[2].data[3]);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[3].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[3].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[3].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[3].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[3].data[3]);
}

void test_repeated_disabled_command_does_not_repeat_idle()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    ActuatorCommand enabledCommand{
        .driveEnabled = true,
        .leftTorque = 1.0f,
        .rightTorque = -1.0f
    };

    ActuatorCommand disabledCommand{
        .driveEnabled = false,
        .leftTorque = 0.0f,
        .rightTorque = 0.0f
    };

    // First enter the enabled state.
    motorController.apply(enabledCommand);

    // Ignore frames generated during enable.
    canBus.frameCount = 0;

    // First disable: must send IDLE to both axes.
    motorController.apply(disabledCommand);

    TEST_ASSERT_EQUAL(4, canBus.frameCount);

    // Second disable: already disabled, no additional frames.
    motorController.apply(disabledCommand);

    TEST_ASSERT_EQUAL(4, canBus.frameCount);
}

void test_enabled_drive_sends_only_torque_on_subsequent_updates()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    ActuatorCommand firstCommand{
        .driveEnabled = true,
        .leftTorque = 1.0f,
        .rightTorque = -1.0f
    };

    motorController.apply(firstCommand);

    // Ignore frames generated while entering CLOSED_LOOP.
    canBus.frameCount = 0;

    ActuatorCommand secondCommand{
        .driveEnabled = true,
        .leftTorque = 2.0f,
        .rightTorque = -2.0f
    };

    motorController.apply(secondCommand);

    TEST_ASSERT_EQUAL(2, canBus.frameCount);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[0].id);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[1].id);
}

void test_update_receives_left_heartbeat()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    canBus.rxFrames[0] = CanFrame{
        .id = OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kHeartbeat),
        .data = {
            0x00, 0x00, 0x00, 0x00,
            0x08, 0x00, 0x00, 0x00
        },
        .length = 8
    };

    canBus.rxFrameCount = 1;

    motorController.update(1234);

    const OdescMotorStatus& status =
        motorController.status();

    TEST_ASSERT_EQUAL_UINT32(
        1234,
        status.left.lastHeartbeatMs);

    TEST_ASSERT_EQUAL_UINT32(
        0,
        status.right.lastHeartbeatMs);

    TEST_ASSERT_TRUE(
        status.left.heartbeatReceived);

    TEST_ASSERT_EQUAL_HEX32(
        0,
        status.left.axisError);

    TEST_ASSERT_EQUAL(
        OdescCanProtocol::kAxisStateClosedLoopControl,
        status.left.axisState);

    // Nothing received from the right axis.
    TEST_ASSERT_FALSE(
        status.right.heartbeatReceived);
}

void test_update_receives_both_heartbeats()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    canBus.rxFrames[0] = CanFrame{
        .id = OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kHeartbeat),
        .data = {
            0x00, 0x00, 0x00, 0x00,
            0x08, 0x00, 0x00, 0x00
        },
        .length = 8
    };

    canBus.rxFrames[1] = CanFrame{
        .id = OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kHeartbeat),
        .data = {
            0x34, 0x12, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00
        },
        .length = 8
    };

    canBus.rxFrameCount = 2;

    motorController.update(1000);

    const OdescMotorStatus& status =
        motorController.status();

    // Left axis
    TEST_ASSERT_TRUE(
        status.left.heartbeatReceived);

    TEST_ASSERT_EQUAL_HEX32(
        0,
        status.left.axisError);

    TEST_ASSERT_EQUAL(
        OdescCanProtocol::kAxisStateClosedLoopControl,
        status.left.axisState);

    // Right axis
    TEST_ASSERT_TRUE(
        status.right.heartbeatReceived);

    TEST_ASSERT_EQUAL_HEX32(
        0x00001234,
        status.right.axisError);

    TEST_ASSERT_EQUAL(
        OdescCanProtocol::kAxisStateIdle,
        status.right.axisState);
}

void test_heartbeat_freshness()
{
    OdescAxisStatus status{
        .heartbeatReceived = true,
        .lastHeartbeatMs = 1000,
        .axisError = 0,
        .axisState = OdescCanProtocol::kAxisStateClosedLoopControl
    };

    TEST_ASSERT_TRUE(
        isHeartbeatFresh(status, 1050, 100));

    TEST_ASSERT_TRUE(
        isHeartbeatFresh(status, 1100, 100));

    TEST_ASSERT_FALSE(
        isHeartbeatFresh(status, 1101, 100));
}

void test_heartbeat_never_received_is_not_fresh()
{
    OdescAxisStatus status{};

    TEST_ASSERT_FALSE(
        isHeartbeatFresh(status, 1000, 100));
}

void test_heartbeat_freshness_handles_uint32_wraparound()
{
    OdescAxisStatus status{
        .heartbeatReceived = true,
        .lastHeartbeatMs = 0xFFFFFFF0,
        .axisError = 0,
        .axisState = OdescCanProtocol::kAxisStateClosedLoopControl
    };

    // 0xFFFFFFF0 -> 0x00000010 = 32 ms
    TEST_ASSERT_TRUE(
        isHeartbeatFresh(status, 0x00000010, 100));

    // 0xFFFFFFF0 -> 0x00000060 = 112 ms
    TEST_ASSERT_FALSE(
        isHeartbeatFresh(status, 0x00000060, 100));
}

void test_axis_health()
{
    OdescAxisStatus status{};

    // No heartbeat received.
    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::UNAVAILABLE),
        static_cast<int>(
            getAxisHealth(status, 1000, 100)));

    // Fresh heartbeat, no error.
    status.heartbeatReceived = true;
    status.lastHeartbeatMs = 1000;
    status.axisError = 0;

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(
            getAxisHealth(status, 1050, 100)));

    // Fresh heartbeat, but ODESC reports an error.
    status.axisError = 0x01;

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::FAULT),
        static_cast<int>(
            getAxisHealth(status, 1050, 100)));

    // Error is present, but heartbeat is now stale.
    status.axisError = 0x01;

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::UNAVAILABLE),
        static_cast<int>(
            getAxisHealth(status, 1200, 100)));
}

void test_motor_health()
{
    OdescMotorStatus status{};

    // No heartbeat from either axis.
    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::UNAVAILABLE),
        static_cast<int>(
            getMotorHealth(status, 1050, 100)));

    // Both axes healthy.
    status.left.heartbeatReceived = true;
    status.left.lastHeartbeatMs = 1000;
    status.left.axisError = 0;

    status.right.heartbeatReceived = true;
    status.right.lastHeartbeatMs = 1000;
    status.right.axisError = 0;

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::OK),
        static_cast<int>(
            getMotorHealth(status, 1050, 100)));

    // One axis unavailable.
    status.right.heartbeatReceived = false;

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::UNAVAILABLE),
        static_cast<int>(
            getMotorHealth(status, 1050, 100)));

    // FAULT has precedence over UNAVAILABLE.
    status.left.axisError = 0x01;

    TEST_ASSERT_EQUAL(
        static_cast<int>(HealthState::FAULT),
        static_cast<int>(
            getMotorHealth(status, 1050, 100)));
}

void test_update_receives_encoder_estimates()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    CanFrame frame{
        .id = OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kGetEncoderEstimates),
        .data = {},
        .length = 8
    };

    OdescCanProtocol::writeFloat32LE(
        frame.data,
        1.5f);

    OdescCanProtocol::writeFloat32LE(
        frame.data + 4,
        2.0f);

    canBus.rxFrames[0] = frame;
    canBus.rxFrameCount = 1;

    motorController.update(1000);

    constexpr float kExpectedSpeed =
        4.0f * 3.14159265358979323846f;

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        kExpectedSpeed,
        motorController.leftWheelSpeedRadS());

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.0f,
        motorController.rightWheelSpeedRadS());
}

void test_update_receives_bus_voltage_current()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

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

    canBus.rxFrames[0] = frame;
    canBus.rxFrameCount = 1;

    motorController.update(1234);

    const OdescMotorStatus& status =
        motorController.status();

    TEST_ASSERT_TRUE(
        status.power.received);

    TEST_ASSERT_EQUAL_UINT32(
        1234,
        status.power.lastUpdateMs);

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        24.5f,
        status.power.busVoltage);

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        3.25f,
        status.power.busCurrent);
}

void test_wheel_speed_freshness()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    // No encoder data received yet.
    TEST_ASSERT_FALSE(
        motorController.wheelSpeedsFresh(
            1000,
            100));

    CanFrame leftFrame{
        .id = OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kGetEncoderEstimates),
        .data = {},
        .length = 8
    };

    OdescCanProtocol::writeFloat32LE(
        leftFrame.data,
        0.0f);
    OdescCanProtocol::writeFloat32LE(
        leftFrame.data + 4,
        1.0f);

    canBus.rxFrames[0] = leftFrame;
    canBus.rxFrameCount = 1;

    motorController.update(1000);

    // Only left encoder received.
    TEST_ASSERT_FALSE(
        motorController.wheelSpeedsFresh(
            1000,
            100));

    CanFrame rightFrame{
        .id = OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kGetEncoderEstimates),
        .data = {},
        .length = 8
    };

    OdescCanProtocol::writeFloat32LE(
        rightFrame.data,
        0.0f);
    OdescCanProtocol::writeFloat32LE(
        rightFrame.data + 4,
        1.0f);

    canBus.rxFrames[1] = rightFrame;

    // Process only the new frame.
    canBus.rxFrameIndex = 1;
    canBus.rxFrameCount = 2;

    motorController.update(1050);

    // Left age = 50 ms, right age = 0 ms.
    TEST_ASSERT_TRUE(
        motorController.wheelSpeedsFresh(
            1050,
            100));

    // Exactly at timeout is still valid.
    TEST_ASSERT_TRUE(
        motorController.wheelSpeedsFresh(
            1100,
            100));

    // Left is now 101 ms old.
    TEST_ASSERT_FALSE(
        motorController.wheelSpeedsFresh(
            1101,
            100));
}

void test_wheel_speed_freshness_handles_uint32_wraparound()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    CanFrame leftFrame{
        .id = OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kGetEncoderEstimates),
        .data = {},
        .length = 8
    };

    OdescCanProtocol::writeFloat32LE(
        leftFrame.data,
        0.0f);
    OdescCanProtocol::writeFloat32LE(
        leftFrame.data + 4,
        1.0f);

    CanFrame rightFrame{
        .id = OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kGetEncoderEstimates),
        .data = {},
        .length = 8
    };

    OdescCanProtocol::writeFloat32LE(
        rightFrame.data,
        0.0f);
    OdescCanProtocol::writeFloat32LE(
        rightFrame.data + 4,
        1.0f);

    canBus.rxFrames[0] = leftFrame;
    canBus.rxFrames[1] = rightFrame;
    canBus.rxFrameCount = 2;

    motorController.update(0xFFFFFFF0);

    // 0xFFFFFFF0 -> 0x00000010 = 32 ms
    TEST_ASSERT_TRUE(
        motorController.wheelSpeedsFresh(
            0x00000010,
            100));

    // 0xFFFFFFF0 -> 0x00000060 = 112 ms
    TEST_ASSERT_FALSE(
        motorController.wheelSpeedsFresh(
            0x00000060,
            100));
}

void test_closed_loop_confirmed_sends_requested_torque()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    CanFrame leftHeartbeat{};
    leftHeartbeat.id =
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kHeartbeat);
    leftHeartbeat.length = 8;
    leftHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateClosedLoopControl);

    CanFrame rightHeartbeat{};
    rightHeartbeat.id =
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kHeartbeat);
    rightHeartbeat.length = 8;
    rightHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateClosedLoopControl);

    canBus.rxFrames[0] = leftHeartbeat;
    canBus.rxFrames[1] = rightHeartbeat;
    canBus.rxFrameCount = 2;

    motorController.update(100);

    ActuatorCommand command{
        .driveEnabled = true,
        .leftTorque = 1.0f,
        .rightTorque = -1.0f
    };

    motorController.apply(command);

    TEST_ASSERT_EQUAL(2, canBus.frameCount);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[0].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[0].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[0].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x80, canBus.frames[0].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x3F, canBus.frames[0].data[3]);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[1].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[1].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[1].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x80, canBus.frames[1].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xBF, canBus.frames[1].data[3]);
}

void test_lost_closed_loop_sends_zero_torque_and_retries_enable()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    CanFrame leftHeartbeat{};
    leftHeartbeat.id =
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kHeartbeat);
    leftHeartbeat.length = 8;
    leftHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateClosedLoopControl);

    CanFrame rightHeartbeat{};
    rightHeartbeat.id =
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kHeartbeat);
    rightHeartbeat.length = 8;
    rightHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateClosedLoopControl);

    canBus.rxFrames[0] = leftHeartbeat;
    canBus.rxFrames[1] = rightHeartbeat;
    canBus.rxFrameCount = 2;

    motorController.update(100);

    ActuatorCommand command{
        .driveEnabled = true,
        .leftTorque = 1.0f,
        .rightTorque = -1.0f
    };

    motorController.apply(command);

    TEST_ASSERT_EQUAL(2, canBus.frameCount);

    // Simulate left axis leaving CLOSED_LOOP_CONTROL.
    CanFrame leftIdleHeartbeat{};
    leftIdleHeartbeat.id =
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kHeartbeat);
    leftIdleHeartbeat.length = 8;
    leftIdleHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateIdle);

    canBus.rxFrames[0] = leftIdleHeartbeat;
    canBus.rxFrameCount = 1;
    canBus.rxFrameIndex = 0;

    motorController.update(200);

    motorController.apply(command);

    TEST_ASSERT_EQUAL(6, canBus.frameCount);

    // CLOSED_LOOP_CONTROL requested again.
    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetAxisRequestedState),
        canBus.frames[2].id);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetAxisRequestedState),
        canBus.frames[3].id);

    // Torque is forced to zero.
    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[4].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[3]);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[5].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[3]);
}

void test_stale_heartbeat_disables_requested_torque()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    CanFrame leftHeartbeat{};
    leftHeartbeat.id =
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kHeartbeat);
    leftHeartbeat.length = 8;
    leftHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateClosedLoopControl);

    CanFrame rightHeartbeat{};
    rightHeartbeat.id =
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kHeartbeat);
    rightHeartbeat.length = 8;
    rightHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateClosedLoopControl);

    canBus.rxFrames[0] = leftHeartbeat;
    canBus.rxFrames[1] = rightHeartbeat;
    canBus.rxFrameCount = 2;

    motorController.update(100);

    ActuatorCommand command{
        .driveEnabled = true,
        .leftTorque = 1.0f,
        .rightTorque = -1.0f
    };

    motorController.apply(command);

    TEST_ASSERT_EQUAL(2, canBus.frameCount);

    // No new heartbeat. At t=301 ms the old heartbeat is stale.
    motorController.update(301);
    motorController.apply(command);

    TEST_ASSERT_EQUAL(6, canBus.frameCount);

    // CLOSED_LOOP_CONTROL is requested again.
    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetAxisRequestedState),
        canBus.frames[2].id);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetAxisRequestedState),
        canBus.frames[3].id);

    // Torque is forced to zero.
    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[4].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[3]);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[5].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[3]);
}

void test_axis_error_disables_requested_torque()
{
    FakeCanBus canBus;

    OdescMotorController motorController(
        canBus,
        1,
        2,
        200);

    CanFrame leftHeartbeat{};
    leftHeartbeat.id =
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kHeartbeat);
    leftHeartbeat.length = 8;
    leftHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateClosedLoopControl);

    CanFrame rightHeartbeat{};
    rightHeartbeat.id =
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kHeartbeat);
    rightHeartbeat.length = 8;
    rightHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateClosedLoopControl);

    canBus.rxFrames[0] = leftHeartbeat;
    canBus.rxFrames[1] = rightHeartbeat;
    canBus.rxFrameCount = 2;

    motorController.update(100);

    ActuatorCommand command{
        .driveEnabled = true,
        .leftTorque = 1.0f,
        .rightTorque = -1.0f
    };

    motorController.apply(command);

    TEST_ASSERT_EQUAL(2, canBus.frameCount);

    // New heartbeat: left axis reports an error while still
    // declaring CLOSED_LOOP_CONTROL.
    CanFrame leftErrorHeartbeat{};
    leftErrorHeartbeat.id =
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kHeartbeat);
    leftErrorHeartbeat.length = 8;

    // axisError = 1, little endian.
    leftErrorHeartbeat.data[0] = 0x01;
    leftErrorHeartbeat.data[1] = 0x00;
    leftErrorHeartbeat.data[2] = 0x00;
    leftErrorHeartbeat.data[3] = 0x00;

    leftErrorHeartbeat.data[4] =
        static_cast<uint8_t>(
            OdescCanProtocol::kAxisStateClosedLoopControl);

    canBus.rxFrames[0] = leftErrorHeartbeat;
    canBus.rxFrameCount = 1;
    canBus.rxFrameIndex = 0;

    motorController.update(200);
    motorController.apply(command);

    TEST_ASSERT_EQUAL(6, canBus.frameCount);

    // CLOSED_LOOP_CONTROL is requested again.
    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetAxisRequestedState),
        canBus.frames[2].id);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetAxisRequestedState),
        canBus.frames[3].id);

    // Requested torque must not be forwarded.
    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            1,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[4].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[4].data[3]);

    TEST_ASSERT_EQUAL_HEX32(
        OdescCanProtocol::makeCanId(
            2,
            OdescCanProtocol::kSetInputTorque),
        canBus.frames[5].id);

    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, canBus.frames[5].data[3]);
}

void test_power_status_freshness()
{
    OdescPowerStatus status{};

    // Never received.
    TEST_ASSERT_FALSE(
        isPowerStatusFresh(
            status,
            1000,
            100));

    status.received = true;
    status.lastUpdateMs = 1000;

    TEST_ASSERT_TRUE(
        isPowerStatusFresh(
            status,
            1050,
            100));

    // Exactly at timeout is still fresh.
    TEST_ASSERT_TRUE(
        isPowerStatusFresh(
            status,
            1100,
            100));

    TEST_ASSERT_FALSE(
        isPowerStatusFresh(
            status,
            1101,
            100));
}

void test_power_status_freshness_handles_uint32_wraparound()
{
    OdescPowerStatus status{
        .received = true,
        .lastUpdateMs = 0xFFFFFFF0,
        .busVoltage = 24.0f,
        .busCurrent = 2.0f
    };

    // 0xFFFFFFF0 -> 0x00000010 = 32 ms
    TEST_ASSERT_TRUE(
        isPowerStatusFresh(
            status,
            0x00000010,
            100));

    // 0xFFFFFFF0 -> 0x00000060 = 112 ms
    TEST_ASSERT_FALSE(
        isPowerStatusFresh(
            status,
            0x00000060,
            100));
}


void test_torque_safety_clamp()
{
    FakeCanBus canBus;
    OdescMotorController motorController(
        canBus, 1, 2, 200);

    // Simulate both axes in CLOSED_LOOP_CONTROL.
    for (uint8_t nodeId = 1; nodeId <= 2; ++nodeId)
    {
        CanFrame frame{};
        frame.id = OdescCanProtocol::makeCanId(
            nodeId, OdescCanProtocol::kHeartbeat);
        frame.length = 8;
        frame.data[4] =
            OdescCanProtocol::kAxisStateClosedLoopControl;

        canBus.rxFrames[canBus.rxFrameCount++] = frame;
    }

    motorController.update(100);

    motorController.apply({true, 7.0f, -8.0f});

    TEST_ASSERT_EQUAL(2, canBus.frameCount);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 5.0f,
        OdescCanProtocol::readFloat32LE(
            canBus.frames[0].data));

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, -5.0f,
        OdescCanProtocol::readFloat32LE(
            canBus.frames[1].data));

    TEST_ASSERT_TRUE(
        motorController.lastApplyResult().torqueClamped);

    TEST_ASSERT_FALSE(
        motorController.lastApplyResult().invalidTorque);
}

void test_non_finite_torque_rejected()
{
    FakeCanBus canBus;
    OdescMotorController motorController(
        canBus, 1, 2, 200);

    // Simulate both axes in CLOSED_LOOP_CONTROL.
    for (uint8_t nodeId = 1; nodeId <= 2; ++nodeId)
    {
        CanFrame frame{};
        frame.id = OdescCanProtocol::makeCanId(
            nodeId, OdescCanProtocol::kHeartbeat);
        frame.length = 8;
        frame.data[4] =
            OdescCanProtocol::kAxisStateClosedLoopControl;

        canBus.rxFrames[canBus.rxFrameCount++] = frame;
    }

    motorController.update(100);

    const float nan =
        std::numeric_limits<float>::quiet_NaN();

    motorController.apply({true, nan, 2.0f});

    TEST_ASSERT_TRUE(
        motorController.lastApplyResult().invalidTorque);

    // No invalid CAN torque command must be transmitted.
    TEST_ASSERT_EQUAL(0, canBus.frameCount);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_initial_disabled_command_sends_nothing);
    RUN_TEST(test_enabling_drive_requests_closed_loop_and_sends_zero_torque);
    RUN_TEST(test_repeated_disabled_command_does_not_repeat_idle);
    RUN_TEST(test_enabled_drive_sends_only_torque_on_subsequent_updates);
    RUN_TEST(test_update_receives_left_heartbeat);
    RUN_TEST(test_update_receives_both_heartbeats);
    RUN_TEST(test_heartbeat_freshness);
    RUN_TEST(test_heartbeat_never_received_is_not_fresh);
    RUN_TEST(test_heartbeat_freshness_handles_uint32_wraparound);
    RUN_TEST(test_axis_health);
    RUN_TEST(test_motor_health);
    RUN_TEST(test_update_receives_encoder_estimates);
    RUN_TEST(test_update_receives_bus_voltage_current);
    RUN_TEST(test_wheel_speed_freshness);
    RUN_TEST(test_wheel_speed_freshness_handles_uint32_wraparound);
    RUN_TEST(test_closed_loop_confirmed_sends_requested_torque);
    RUN_TEST(test_lost_closed_loop_sends_zero_torque_and_retries_enable);
    RUN_TEST(test_stale_heartbeat_disables_requested_torque);
    RUN_TEST(test_axis_error_disables_requested_torque);
    RUN_TEST(test_power_status_freshness);
    RUN_TEST(test_power_status_freshness_handles_uint32_wraparound);
    RUN_TEST(test_torque_safety_clamp);
    RUN_TEST(test_non_finite_torque_rejected);

    return UNITY_END();
}
