#include <unity.h>

#include "../../src/control/RobotController.h"
#include "../../src/control/ActuatorCommand.h"

namespace
{
constexpr float kPi = 3.14159265358979323846f;
}

void test_idle_produces_zero_torque()
{
    ControlParams params{
        .kpBalance = 2.0f,
        .kdBalance = 0.5f,
        .kpSpeed = 0.0f,
        .kiSpeed = 0.0f,
        .maxSpeedIntegralTorque = 0.0f,
        .kpYaw = 0.0f,
        .kdYaw = 0.0f,
        .wheelRadius = 0.5f,
        .maxTorque = 100.0f
    };

    RobotController controller(params);

    RobotState state{
        .pitch = 10.0f,
        .pitchRate = 0.0f,
        .yaw = 0.0f,
        .yawRate = 0.0f,
        .leftWheelSpeed = 0.0f,
        .rightWheelSpeed = 0.0f
    };

    MotionCommand command{
        .velocity = 0.0f,
        .steering = 0.0f
    };

    SafetyStatus safety{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    const ActuatorCommand output =
        controller.update(state, command, safety, 0.01f);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(controller.mode()));

    TEST_ASSERT_FALSE(output.driveEnabled);
    
    // Despite the 10 degree pitch error, motors must receive no torque
    // because the robot is not ACTIVE.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.rightTorque);
}

void test_active_produces_balance_torque()
{
    ControlParams params{
        .kpBalance = 2.0f,
        .kdBalance = 0.0f,
        .kpSpeed = 0.0f,
        .kiSpeed = 0.0f,
        .maxSpeedIntegralTorque = 0.0f,
        .kpYaw = 0.0f,
        .kdYaw = 0.0f,
        .wheelRadius = 0.5f,
        .maxTorque = 100.0f
    };

    RobotController controller(params);

    RobotState state{
        .pitch = 10.0f,
        .pitchRate = 0.0f,
        .yaw = 0.0f,
        .yawRate = 0.0f,
        .leftWheelSpeed = 0.0f,
        .rightWheelSpeed = 0.0f
    };

    MotionCommand command{
        .velocity = 0.0f,
        .steering = 0.0f
    };

    SafetyStatus safety{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    controller.update(state, command, safety, 0.01f);

    // IDLE -> ACTIVE
    safety.enableRequested = true;

    const ActuatorCommand output =
        controller.update(state, command, safety, 0.01f);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(controller.mode()));

    TEST_ASSERT_TRUE(output.driveEnabled);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 20.0f, output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 20.0f, output.rightTorque);
}

void test_safe_stop_immediately_produces_zero_torque()
{
    ControlParams params{
        .kpBalance = 2.0f,
        .kdBalance = 0.0f,
        .kpSpeed = 0.0f,
        .kiSpeed = 0.0f,
        .maxSpeedIntegralTorque = 0.0f,
        .kpYaw = 0.0f,
        .kdYaw = 0.0f,
        .wheelRadius = 0.5f,
        .maxTorque = 100.0f
    };

    RobotController controller(params);

    RobotState state{
        .pitch = 10.0f,
        .pitchRate = 0.0f,
        .yaw = 0.0f,
        .yawRate = 0.0f,
        .leftWheelSpeed = 0.0f,
        .rightWheelSpeed = 0.0f
    };

    MotionCommand command{
        .velocity = 0.0f,
        .steering = 0.0f
    };

    SafetyStatus safety{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    controller.update(state, command, safety, 0.01f);

    // IDLE -> ACTIVE
    safety.enableRequested = true;

    ActuatorCommand output =
        controller.update(state, command, safety, 0.01f);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(controller.mode()));

    // Verify that the controller is actually producing torque.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 20.0f, output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 20.0f, output.rightTorque);

    // Safety condition occurs.
    safety.tiltExceeded = true;

    output =
        controller.update(state, command, safety, 0.01f);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(controller.mode()));

    TEST_ASSERT_FALSE(output.driveEnabled);

    // Torque must disappear in this same update cycle.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.rightTorque);
}

void test_fault_immediately_produces_zero_torque()
{
    ControlParams params{
        .kpBalance = 2.0f,
        .kdBalance = 0.0f,
        .kpSpeed = 0.0f,
        .kiSpeed = 0.0f,
        .maxSpeedIntegralTorque = 0.0f,
        .kpYaw = 0.0f,
        .kdYaw = 0.0f,
        .wheelRadius = 0.5f,
        .maxTorque = 100.0f
    };

    RobotController controller(params);

    RobotState state{
        .pitch = 10.0f,
        .pitchRate = 0.0f,
        .yaw = 0.0f,
        .yawRate = 0.0f,
        .leftWheelSpeed = 0.0f,
        .rightWheelSpeed = 0.0f
    };

    MotionCommand command{
        .velocity = 0.0f,
        .steering = 0.0f
    };

    SafetyStatus safety{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    controller.update(state, command, safety, 0.01f);

    // IDLE -> ACTIVE
    safety.enableRequested = true;

    ActuatorCommand output =
        controller.update(state, command, safety, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 20.0f, output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 20.0f, output.rightTorque);

    // ODESC reports a hard fault.
    safety.motorController = HealthState::FAULT;

    output =
        controller.update(state, command, safety, 0.01f);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::FAULT),
        static_cast<int>(controller.mode()));

    TEST_ASSERT_FALSE(output.driveEnabled);
    
    // Torque must disappear in this same update cycle.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.rightTorque);
}

void test_rearm_resets_yaw_reference_to_current_heading()
{
    ControlParams params{
        .kpBalance = 0.0f,
        .kdBalance = 0.0f,
        .kpSpeed = 0.0f,
        .kiSpeed = 0.0f,
        .maxSpeedIntegralTorque = 0.0f,
        .kpYaw = 2.0f,
        .kdYaw = 0.0f,
        .wheelRadius = 0.5f,
        .maxTorque = 100.0f
    };

    RobotController controller(params);

    RobotState state{
        .pitch = 0.0f,
        .pitchRate = 0.0f,
        .yaw = 30.0f * kPi / 180.0f,
        .yawRate = 0.0f,
        .leftWheelSpeed = 0.0f,
        .rightWheelSpeed = 0.0f
    };

    MotionCommand command{
        .velocity = 0.0f,
        .steering = 0.0f
    };

    SafetyStatus safety{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    controller.update(state, command, safety, 0.01f);

    // IDLE -> ACTIVE.
    // yawReference becomes 30 degrees.
    safety.enableRequested = true;
    controller.update(state, command, safety, 0.01f);

    // ACTIVE -> SAFE_STOP.
    safety.tiltExceeded = true;
    controller.update(state, command, safety, 0.01f);

    // While inactive, the robot is physically rotated.
    state.yaw = 80.0f * kPi / 180.0f;

    // SAFE_STOP -> IDLE.
    safety.tiltExceeded = false;
    safety.resetRequested = true;
    controller.update(state, command, safety, 0.01f);

    // Release reset and enable to satisfy the re-arm interlock.
    safety.resetRequested = false;
    safety.enableRequested = false;
    controller.update(state, command, safety, 0.01f);

    // Explicitly re-arm.
    safety.enableRequested = true;

    const ActuatorCommand output =
        controller.update(state, command, safety, 0.01f);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(controller.mode()));

    // On entering ACTIVE, BalanceController must reset and capture
    // the current yaw (80 degrees) as its new yaw reference.
    // Therefore there must be no steering torque.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.rightTorque);
}

void test_control_params_validation()
{
    ControlParams initial{};
    initial.wheelRadius = 0.08255f;
    initial.maxTorque = 5.0f;

    RobotController controller(initial);

    // Aggiornamento valido
    ControlParams updated = initial;
    updated.kpBalance = 2.5f;

    RobotState state{};
    MotionCommand command{};

    SafetyStatus safety{};
    safety.imu = HealthState::OK;
    safety.motorController = HealthState::OK;
    safety.wheelFeedback = HealthState::OK;
    safety.control = HealthState::OK;

    // BOOT -> IDLE
    controller.update(state, command, safety, 0.001f);

    TEST_ASSERT_EQUAL(
        RobotMode::IDLE,
        controller.mode());
        
    TEST_ASSERT_TRUE(
        controller.setControlParams(updated));

    TEST_ASSERT_EQUAL_FLOAT(
        2.5f,
        controller.controlParams().kpBalance);

    // Raggio ruota non valido
    updated.wheelRadius = 0.0f;

    TEST_ASSERT_FALSE(
        controller.setControlParams(updated));

    // La configurazione precedente deve restare invariata
    TEST_ASSERT_EQUAL_FLOAT(
        0.08255f,
        controller.controlParams().wheelRadius);

    TEST_ASSERT_EQUAL_FLOAT(
        2.5f,
        controller.controlParams().kpBalance);
}

void test_control_params_rejected_while_active()
{
    ControlParams params{};
    params.wheelRadius = 0.08255f;
    params.maxTorque = 5.0f;
    params.kpBalance = 1.0f;

    RobotController controller(params);

    RobotState state{};
    MotionCommand command{};

    SafetyStatus safety{};
    safety.imu = HealthState::OK;
    safety.motorController = HealthState::OK;
    safety.wheelFeedback = HealthState::OK;
    safety.control = HealthState::OK;
    safety.enableRequested = false;
    safety.tiltExceeded = false;

    // BOOT -> IDLE
    controller.update(state, command, safety, 0.001f);

    TEST_ASSERT_EQUAL(
        RobotMode::IDLE,
        controller.mode());

    // IDLE -> ACTIVE
    safety.enableRequested = true;
    controller.update(state, command, safety, 0.001f);

    TEST_ASSERT_EQUAL(
        RobotMode::ACTIVE,
        controller.mode());

    // Tentativo di modificare i guadagni
    ControlParams updated = params;
    updated.kpBalance = 10.0f;

    TEST_ASSERT_FALSE(
        controller.setControlParams(updated));

    // I parametri devono essere rimasti invariati
    TEST_ASSERT_EQUAL_FLOAT(
        1.0f,
        controller.controlParams().kpBalance);
}

void test_control_params_rejected_during_boot()
{
    ControlParams params{};
    params.wheelRadius = 0.08255f;
    params.maxTorque = 5.0f;

    RobotController controller(params);

    ControlParams updated = params;
    updated.kpBalance = 3.0f;

    TEST_ASSERT_EQUAL(
        RobotMode::BOOT,
        controller.mode());

    TEST_ASSERT_FALSE(
        controller.setControlParams(updated));

    TEST_ASSERT_EQUAL_FLOAT(
        params.kpBalance,
        controller.controlParams().kpBalance);
}


void test_control_fault_disables_motors()
{
    ControlParams params{};
    params.wheelRadius = 0.08255f;
    params.maxTorque = 5.0f;

    RobotController controller(params);

    RobotState state{};
    MotionCommand command{};

    SafetyStatus safety{};
    safety.imu = HealthState::OK;
    safety.motorController = HealthState::OK;
    safety.wheelFeedback = HealthState::OK;
    safety.control = HealthState::OK;

    // BOOT -> IDLE
    controller.update(state, command, safety, 0.001f);
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(controller.mode()));

    // IDLE -> ACTIVE
    safety.enableRequested = true;
    controller.update(state, command, safety, 0.001f);
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(controller.mode()));

    // ACTIVE -> FAULT
    safety.control = HealthState::FAULT;

    const ActuatorCommand output =
        controller.update(state, command, safety, 0.001f);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::FAULT),
        static_cast<int>(controller.mode()));

    TEST_ASSERT_FALSE(output.driveEnabled);
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.leftTorque);
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, output.rightTorque);
}


int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_idle_produces_zero_torque);
    RUN_TEST(test_active_produces_balance_torque);
    RUN_TEST(test_safe_stop_immediately_produces_zero_torque);
    RUN_TEST(test_fault_immediately_produces_zero_torque);
    RUN_TEST(test_rearm_resets_yaw_reference_to_current_heading);
    RUN_TEST(test_control_params_validation);
    RUN_TEST(test_control_params_rejected_while_active);
    RUN_TEST(test_control_params_rejected_during_boot);
    RUN_TEST(test_control_fault_disables_motors);

    return UNITY_END();
}
