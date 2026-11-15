#include <unity.h>

#include "../../src/control/BalanceController.h"

namespace
{
constexpr float kPi = 3.14159265358979323846f;
}

void test_zero_state_produces_zero_torque()
{
    ControlParams params{
        .kpBalance = 2.0f,
        .kdBalance = 0.5f,
        .kpSpeed = 1.0f,
        .kiSpeed = 0.1f,
        .maxSpeedIntegralTorque = 1.0f,
        .kpYaw = 1.0f,
        .kdYaw = 0.1f,
        .wheelRadius = 0.5f,
        .maxTorque = 100.0f
    };

    BalanceController controller(params);

    RobotState state{
        .pitch = 0.0f,
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

    const MotorCommand output =
        controller.update(state, command, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.leftTorque);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.rightTorque);
}

void test_balance_pd_produces_expected_torque()
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

    BalanceController controller(params);

    RobotState state{
        .pitch = 3.0f,
        .pitchRate = -2.0f,
        .yaw = 0.0f,
        .yawRate = 0.0f,
        .leftWheelSpeed = 0.0f,
        .rightWheelSpeed = 0.0f
    };

    MotionCommand command{
        .velocity = 0.0f,
        .steering = 0.0f
    };

    const MotorCommand output =
        controller.update(state, command, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 5.0f, output.leftTorque);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 5.0f, output.rightTorque);
}

void test_speed_pi_uses_dt()
{
    ControlParams params{
        .kpBalance = 0.0f,
        .kdBalance = 0.0f,
        .kpSpeed = 1.0f,
        .kiSpeed = 2.0f,
        .maxSpeedIntegralTorque = 10.0f,
        .kpYaw = 0.0f,
        .kdYaw = 0.0f,
        .wheelRadius = 0.5f,
        .maxTorque = 100.0f
    };

    BalanceController controller(params);

    RobotState state{
        .pitch = 0.0f,
        .pitchRate = 0.0f,
        .yaw = 0.0f,
        .yawRate = 0.0f,
        .leftWheelSpeed = 2.0f,
        .rightWheelSpeed = 2.0f
    };

    MotionCommand command{
        .velocity = 0.0f,
        .steering = 0.0f
    };

    const MotorCommand output =
        controller.update(state, command, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.306f, output.leftTorque);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.306f, output.rightTorque);
}

void test_speed_integral_anti_windup()
{
    ControlParams params{
        .kpBalance = 0.0f,
        .kdBalance = 0.0f,
        .kpSpeed = 0.0f,
        .kiSpeed = 2.0f,
        .maxSpeedIntegralTorque = 1.5f,
        .kpYaw = 0.0f,
        .kdYaw = 0.0f,
        .wheelRadius = 0.5f,
        .maxTorque = 100.0f
    };

    BalanceController controller(params);

    RobotState state{
        .pitch = 0.0f,
        .pitchRate = 0.0f,
        .yaw = 0.0f,
        .yawRate = 0.0f,
        .leftWheelSpeed = 10.0f,
        .rightWheelSpeed = 10.0f
    };

    MotionCommand command{
        .velocity = 0.0f,
        .steering = 0.0f
    };

    MotorCommand output{};

    // Keep a large speed error for long enough to saturate
    // the integral contribution.
    for (int i = 0; i < 1000; ++i)
    {
        output = controller.update(state, command, 0.01f);
    }

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        1.5f,
        output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        1.5f,
        output.rightTorque);
}

void test_yaw_disturbance_produces_restoring_torque()
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

    BalanceController controller(params);

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

    // Initializes yawReference_ to 30 degrees.
    controller.update(state, command, 0.01f);

    // External disturbance rotates the robot by +5 degrees.
    state.yaw = 35.0f * kPi / 180.0f;

    const MotorCommand output =
        controller.update(state, command, 0.01f);

    // yawError = 35 - 30 = +5 degrees
    // steeringTorque = kpYaw * yawError = 2 * 5 = 10
    //
    // left  = -steeringTorque
    // right = +steeringTorque

    const float expectedTorque =
        2.0f * (5.0f * kPi / 180.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        -expectedTorque,
        output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        expectedTorque,
        output.rightTorque);
}

void test_steering_updates_yaw_reference()
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

    BalanceController controller(params);

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
        .steering = 0.5f  // rad/s desired yaw rate
    };

    const MotorCommand output =
        controller.update(state, command, 0.1f);

    // desiredYawRate = 0.5 rad/s
    // yawReference moves by:
    //   0.5 * 0.1 = 0.05 rad
    //
    // yawError = -0.05 rad
    // steeringTorque = 2 * -0.05 = -0.1
    //
    // left  = -steeringTorque = +0.1
    // right = +steeringTorque = -0.1

    const float expectedTorque = 0.1f;

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        expectedTorque,
        output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        -expectedTorque,
        output.rightTorque);
}

void test_yaw_wrap_uses_shortest_angle()
{
    ControlParams params{
        .kpBalance = 0.0f,
        .kdBalance = 0.0f,
        .kpSpeed = 0.0f,
        .kiSpeed = 0.0f,
        .maxSpeedIntegralTorque = 0.0f,
        .kpYaw = 1.0f,
        .kdYaw = 0.0f,
        .wheelRadius = 0.5f,
        .maxTorque = 100.0f
    };

    BalanceController controller(params);

    RobotState state{
        .pitch = 0.0f,
        .pitchRate = 0.0f,
        .yaw = 179.0f * kPi / 180.0f,
        .yawRate = 0.0f,
        .leftWheelSpeed = 0.0f,
        .rightWheelSpeed = 0.0f
    };

    MotionCommand command{
        .velocity = 0.0f,
        .steering = 0.0f
    };

    // Initialize yawReference_ to +179 degrees.
    controller.update(state, command, 0.01f);

    // Cross the +180/-180 boundary.
    state.yaw = -179.0f * kPi / 180.0f;

    const MotorCommand output =
        controller.update(state, command, 0.01f);

    // Raw error:
    //   -179 - (+179) = -358 degrees
    //
    // Normalized error:
    //   +2 degrees
    //
    // steeringTorque = 1 * 2 = 2
    //
    // left  = -2
    // right = +2

    const float expectedYawError =
        2.0f * kPi / 180.0f;

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -expectedYawError, output.leftTorque);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f,  expectedYawError, output.rightTorque);
}

void test_reset_reinitializes_yaw_reference()
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

    BalanceController controller(params);

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

    // yawReference_ becomes 30 degrees.
    controller.update(state, command, 0.01f);

    // Robot is moved while the controller is inactive.
    state.yaw = 80.0f * kPi / 180.0f;

    controller.reset();

    const MotorCommand output =
        controller.update(state, command, 0.01f);

    // After reset, the current yaw (80 degrees) must become
    // the new reference. Therefore yaw error is zero.
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.leftTorque);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.rightTorque);
}

void test_output_torque_is_limited()
{
    ControlParams params{
        .kpBalance = 10.0f,
        .kdBalance = 0.0f,
        .kpSpeed = 0.0f,
        .kiSpeed = 0.0f,
        .maxSpeedIntegralTorque = 0.0f,
        .kpYaw = 0.0f,
        .kdYaw = 0.0f,
        .wheelRadius = 0.5f,
        .maxTorque = 2.0f
    };

    BalanceController controller(params);

    RobotState state{
        .pitch = 1.0f,
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

    const MotorCommand output =
        controller.update(state, command, 0.01f);

    // Raw balance torque would be +10 N·m,
    // therefore both outputs must saturate at +2 N·m.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 2.0f, output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 2.0f, output.rightTorque);

    state.pitch = -1.0f;

    const MotorCommand negativeOutput =
        controller.update(state, command, 0.01f);

    // Raw balance torque would be -10 N·m,
    // therefore both outputs must saturate at -2 N·m.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, -2.0f, negativeOutput.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, -2.0f, negativeOutput.rightTorque);
}

void test_set_params_updates_gains_and_resets_integrator()
{
    ControlParams initial{};
    initial.kpSpeed = 0.0f;
    initial.kiSpeed = 1.0f;
    initial.maxSpeedIntegralTorque = 10.0f;
    initial.wheelRadius = 0.1f;
    initial.maxTorque = 20.0f;

    BalanceController controller(initial);

    RobotState state{};
    state.leftWheelSpeed = 10.0f;
    state.rightWheelSpeed = 10.0f;

    MotionCommand command{};

    // Accumula errore integrale.
    controller.update(state, command, 1.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        0.3f,
        controller.diagnostics().speedIntegralTorque);

    // Aggiorna i parametri: deve resettare lo stato interno.
    ControlParams updated = initial;
    updated.kpSpeed = 2.0f;

    controller.setParams(updated);

    TEST_ASSERT_EQUAL_FLOAT(
        2.0f,
        controller.params().kpSpeed);

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        0.0f,
        controller.diagnostics().speedIntegralTorque);

    // Verifica che il nuovo Kp venga utilizzato.
    const MotorCommand output =
        controller.update(state, command, 1.0f);

    // Dopo reset, errore filtrato = 0.3 m/s,
    // contributo P = 2 * 0.3 = 0.6 Nm,
    // contributo I = 1 * 0.3 = 0.3 Nm.
    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        0.9f,
        output.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        0.9f,
        output.rightTorque);
}

void test_set_params_resets_yaw_reference()
{
    ControlParams params{};
    params.kpYaw = 2.0f;
    params.maxTorque = 10.0f;
    params.wheelRadius = 0.1f;

    BalanceController controller(params);

    RobotState state{};
    MotionCommand command{};

    // Inizializza il riferimento yaw a zero.
    controller.update(state, command, 0.01f);

    // Cambia orientamento: deve comparire una coppia di sterzata.
    state.yaw = 0.5f;

    MotorCommand before =
        controller.update(state, command, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f, 1.0f, before.rightTorque);

    // Aggiorna i parametri e resetta lo stato interno.
    controller.setParams(params);

    MotorCommand after =
        controller.update(state, command, 0.01f);

    // Il nuovo riferimento yaw deve essere 0.5 rad.
    // Quindi l'errore yaw è zero.
    TEST_ASSERT_FLOAT_WITHIN(
        0.001f, 0.0f, after.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f, 0.0f, after.rightTorque);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_zero_state_produces_zero_torque);
    RUN_TEST(test_balance_pd_produces_expected_torque);
    RUN_TEST(test_speed_pi_uses_dt);
    RUN_TEST(test_speed_integral_anti_windup);
    RUN_TEST(test_yaw_disturbance_produces_restoring_torque);
    RUN_TEST(test_steering_updates_yaw_reference);
    RUN_TEST(test_yaw_wrap_uses_shortest_angle);
    RUN_TEST(test_reset_reinitializes_yaw_reference);
    RUN_TEST(test_output_torque_is_limited);
    RUN_TEST(test_set_params_updates_gains_and_resets_integrator);
    RUN_TEST(test_set_params_resets_yaw_reference);

    return UNITY_END();
}