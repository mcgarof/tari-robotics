#include <unity.h>

#include "../../src/safety/SafetyManager.h"

void test_boot_transitions_to_idle_when_hardware_is_healthy()
{
    SafetyManager safety;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::BOOT),
        static_cast<int>(safety.mode()));

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(mode));
}

void test_idle_transitions_to_active_when_enabled_and_safe()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(safety.mode()));

    // Request activation.
    status.enableRequested = true;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(mode));
}

void test_idle_does_not_activate_when_tilt_exceeded()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // Try to activate while the robot is outside the safe tilt range.
    status.enableRequested = true;
    status.tiltExceeded = true;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(mode));
}

void test_active_transitions_to_safe_stop_when_tilt_exceeded()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(safety.mode()));

    // Robot exceeds the safe tilt range.
    status.tiltExceeded = true;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(mode));
}

void test_active_transitions_to_safe_stop_when_imu_unhealthy()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(safety.mode()));

    // IMU becomes unavailable or invalid.
    status.imu = HealthState::UNAVAILABLE;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(mode));
}

void test_active_transitions_to_safe_stop_when_motor_controller_unhealthy()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(safety.mode()));

    // Motor controller becomes unavailable or unhealthy.
    status.motorController = HealthState::UNAVAILABLE;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(mode));
}

void test_safe_stop_does_not_recover_automatically()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    // ACTIVE -> SAFE_STOP
    status.tiltExceeded = true;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(safety.mode()));

    // Physical condition becomes safe again,
    // but no explicit reset has been requested.
    status.tiltExceeded = false;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(mode));
}

void test_safe_stop_recovers_to_idle_on_explicit_reset()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    // ACTIVE -> SAFE_STOP
    status.tiltExceeded = true;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(safety.mode()));

    // Conditions become safe again and an explicit reset is requested.
    status.tiltExceeded = false;
    status.resetRequested = true;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(mode));
}

void test_safe_stop_requires_enable_release_before_rearming()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    // ACTIVE -> SAFE_STOP
    status.tiltExceeded = true;
    safety.update(status);

    // Recover, but enable is still asserted.
    status.tiltExceeded = false;
    status.resetRequested = true;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(safety.mode()));

    // Remove reset, but keep enable asserted.
    // Robot must NOT immediately re-arm.
    status.resetRequested = false;

    RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(mode));

    // Release enable.
    status.enableRequested = false;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(safety.mode()));

    // Explicitly enable again.
    status.enableRequested = true;

    mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(mode));
}

void test_active_transitions_to_fault_when_imu_fault()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(safety.mode()));

    // IMU reports a hard fault.
    status.imu = HealthState::FAULT;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::FAULT),
        static_cast<int>(mode));
}

void test_active_transitions_to_fault_when_motor_controller_fault()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(safety.mode()));

    // Motor controller reports a hard fault.
    status.motorController = HealthState::FAULT;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::FAULT),
        static_cast<int>(mode));
}

void test_fault_does_not_recover_automatically()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    // ACTIVE -> FAULT
    status.motorController = HealthState::FAULT;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::FAULT),
        static_cast<int>(safety.mode()));

    // The hardware reports OK again.
    status.motorController = HealthState::OK;

    const RobotMode mode = safety.update(status);

    // FAULT must remain latched.
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::FAULT),
        static_cast<int>(mode));
}

void test_fault_recovers_to_idle_on_explicit_reset()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    // ACTIVE -> FAULT
    status.motorController = HealthState::FAULT;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::FAULT),
        static_cast<int>(safety.mode()));

    // Fault condition disappears.
    status.motorController = HealthState::OK;

    // Explicit reset is requested.
    status.resetRequested = true;

    const RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(mode));
}

void test_fault_requires_enable_release_before_rearming()
{
    SafetyManager safety;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    safety.update(status);

    // IDLE -> ACTIVE
    status.enableRequested = true;
    safety.update(status);

    // ACTIVE -> FAULT
    status.motorController = HealthState::FAULT;
    safety.update(status);

    // Fault disappears and explicit reset is requested.
    // Enable is intentionally still asserted.
    status.motorController = HealthState::OK;
    status.resetRequested = true;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(safety.mode()));

    // Remove reset, but keep enable asserted.
    status.resetRequested = false;

    RobotMode mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(mode));

    // Release enable.
    status.enableRequested = false;
    safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(safety.mode()));

    // Explicitly enable again.
    status.enableRequested = true;

    mode = safety.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(mode));
}

void test_boot_requires_enable_release_before_activation()
{
    SafetyManager manager;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = true,
        .resetRequested = false
    };

    // Robot boots while enable is already held.
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(manager.update(status)));

    // Still held: must NOT activate.
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(manager.update(status)));

    // Release enable.
    status.enableRequested = false;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(manager.update(status)));

    // A new explicit enable request may now activate the robot.
    status.enableRequested = true;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(manager.update(status)));
}

void test_control_unavailable_while_active_enters_safe_stop()
{
    SafetyManager manager;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .wheelFeedback = HealthState::OK,
        .control = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(manager.update(status)));

    // IDLE -> ACTIVE
    status.enableRequested = true;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(manager.update(status)));

    // Control loop becomes unhealthy.
    status.control = HealthState::UNAVAILABLE;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(manager.update(status)));
}

void test_control_must_be_ok_before_activation()
{
    SafetyManager manager;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .control = HealthState::UNAVAILABLE,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // Control is unavailable: BOOT must not complete.
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::BOOT),
        static_cast<int>(manager.update(status)));

    // Control becomes healthy.
    status.control = HealthState::OK;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(manager.update(status)));

    // A new enable request may now activate the robot.
    status.enableRequested = true;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(manager.update(status)));
}

void test_control_must_remain_ok_when_enabling()
{
    SafetyManager manager;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .control = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(manager.update(status)));

    // Control becomes unavailable before Enable.
    status.control = HealthState::UNAVAILABLE;
    status.enableRequested = true;

    // Must remain IDLE.
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(manager.update(status)));
}

void test_reset_requires_control_ok()
{
    SafetyManager manager;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .control = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE -> ACTIVE
    manager.update(status);

    status.enableRequested = true;
    manager.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(manager.mode()));

    // Lose control-loop health -> SAFE_STOP.
    status.control = HealthState::UNAVAILABLE;
    manager.update(status);

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(manager.mode()));

    // Reset while control is still unavailable must not recover.
    status.enableRequested = false;
    status.resetRequested = true;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(manager.update(status)));

    // Once control is healthy, reset may recover to IDLE.
    status.control = HealthState::OK;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(manager.update(status)));
}

void test_wheel_feedback_unavailable_while_active_enters_safe_stop()
{
    SafetyManager manager;

    SafetyStatus status{
        .imu = HealthState::OK,
        .motorController = HealthState::OK,
        .wheelFeedback = HealthState::OK,
        .control = HealthState::OK,
        .tiltExceeded = false,
        .enableRequested = false,
        .resetRequested = false
    };

    // BOOT -> IDLE
    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::IDLE),
        static_cast<int>(manager.update(status)));

    // IDLE -> ACTIVE
    status.enableRequested = true;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::ACTIVE),
        static_cast<int>(manager.update(status)));

    // Wheel feedback becomes unavailable.
    status.wheelFeedback = HealthState::UNAVAILABLE;

    TEST_ASSERT_EQUAL(
        static_cast<int>(RobotMode::SAFE_STOP),
        static_cast<int>(manager.update(status)));
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_boot_transitions_to_idle_when_hardware_is_healthy);
    RUN_TEST(test_idle_transitions_to_active_when_enabled_and_safe);
    RUN_TEST(test_idle_does_not_activate_when_tilt_exceeded);
    RUN_TEST(test_active_transitions_to_safe_stop_when_tilt_exceeded);
    RUN_TEST(test_active_transitions_to_safe_stop_when_imu_unhealthy);
    RUN_TEST(test_active_transitions_to_safe_stop_when_motor_controller_unhealthy);
    RUN_TEST(test_safe_stop_does_not_recover_automatically);
    RUN_TEST(test_safe_stop_recovers_to_idle_on_explicit_reset);
    RUN_TEST(test_safe_stop_requires_enable_release_before_rearming);
    RUN_TEST(test_active_transitions_to_fault_when_imu_fault);
    RUN_TEST(test_active_transitions_to_fault_when_motor_controller_fault);
    RUN_TEST(test_fault_does_not_recover_automatically);
    RUN_TEST(test_fault_recovers_to_idle_on_explicit_reset);
    RUN_TEST(test_fault_requires_enable_release_before_rearming);
    RUN_TEST(test_boot_requires_enable_release_before_activation);
    RUN_TEST(test_control_unavailable_while_active_enters_safe_stop);
    RUN_TEST(test_control_must_be_ok_before_activation);
    RUN_TEST(test_control_must_remain_ok_when_enabling);
    RUN_TEST(test_reset_requires_control_ok);
    RUN_TEST(test_wheel_feedback_unavailable_while_active_enters_safe_stop);

    return UNITY_END();
}
