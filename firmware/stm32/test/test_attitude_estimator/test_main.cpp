#include <unity.h>

#include "control/AttitudeEstimator.h"

void test_stationary_upright_robot_has_zero_attitude()
{
    AttitudeEstimator estimator;

    const ImuSample sample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    const AttitudeState state =
        estimator.update(sample, 0.001f);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, state.pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, state.pitchRate);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, state.yaw);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, state.yawRate);
}

void test_initial_pitch_is_estimated_from_accelerometer()
{
    constexpr float kPi = 3.14159265358979323846f;

    AttitudeEstimator estimator;

    const ImuSample sample{
        .accelX = 4.903325f,
        .accelY = 0.0f,
        .accelZ = 8.4928f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    const AttitudeState state =
        estimator.update(sample, 0.001f);

    const float expectedPitch =
        30.0f * kPi / 180.0f;

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        expectedPitch,
        state.pitch);
}

void test_gyro_contributes_to_pitch_estimate()
{
    AttitudeEstimator estimator(0.5f);

    const ImuSample initialSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    estimator.update(initialSample, 0.1f);

    const ImuSample rotatingSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 1.0f,
        .gyroZ = 0.0f
    };

    const AttitudeState state =
        estimator.update(rotatingSample, 0.1f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.0833333f,
        state.pitch);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        1.0f,
        state.pitchRate);
}

void test_accelerometer_corrects_pitch_drift()
{
    AttitudeEstimator estimator(0.5f);

    // Initialize upright.
    const ImuSample initialSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    estimator.update(initialSample, 0.1f);

    // Gyro says that pitch moved by 0.1 rad,
    // while accelerometer still says upright.
    const ImuSample sample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 1.0f,
        .gyroZ = 0.0f
    };

    const AttitudeState state =
        estimator.update(sample, 0.1f);

    // gyro prediction = 0.1 rad
    // accelerometer pitch = 0 rad
    // complementary filter:
    // 0.98 * 0.1 + 0.02 * 0 = 0.098 rad
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.0833333f,
        state.pitch);
}

void test_filter_behavior_is_consistent_across_sample_rates()
{
    constexpr float timeConstant = 0.5f;
    constexpr float gyroBias = 0.1f;

    const ImuSample initialSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    const ImuSample biasedSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = gyroBias,
        .gyroZ = 0.0f
    };

    AttitudeEstimator estimator1ms(timeConstant);
    AttitudeEstimator estimator2ms(timeConstant);

    estimator1ms.update(initialSample, 0.001f);
    estimator2ms.update(initialSample, 0.002f);

    AttitudeState state1ms{};
    AttitudeState state2ms{};

    // Simulate the same physical duration: 1 second.
    for (int i = 0; i < 1000; ++i)
    {
        state1ms =
            estimator1ms.update(biasedSample, 0.001f);
    }

    for (int i = 0; i < 500; ++i)
    {
        state2ms =
            estimator2ms.update(biasedSample, 0.002f);
    }

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        state1ms.pitch,
        state2ms.pitch);
}

void test_gyro_is_integrated_into_yaw()
{
    AttitudeEstimator estimator(0.5f);

    const ImuSample initialSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    estimator.update(initialSample, 0.2f);

    const ImuSample rotatingSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.5f
    };

    const AttitudeState state =
        estimator.update(rotatingSample, 0.2f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.1f,
        state.yaw);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.5f,
        state.yawRate);
}

void test_reset_reinitializes_attitude()
{
    constexpr float kPi = 3.14159265358979323846f;

    AttitudeEstimator estimator(0.5f);

    const ImuSample initialSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    estimator.update(initialSample, 0.1f);

    // Accumulate some pitch and yaw.
    const ImuSample movingSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 1.0f,
        .gyroZ = 1.0f
    };

    estimator.update(movingSample, 0.1f);

    estimator.reset();

    // After reset, the robot is at a new physical pitch: +30 degrees.
    const ImuSample newSample{
        .accelX = 4.903325f,
        .accelY = 0.0f,
        .accelZ = 8.4928f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    const AttitudeState state =
        estimator.update(newSample, 0.1f);

    const float expectedPitch =
        30.0f * kPi / 180.0f;

    TEST_ASSERT_FLOAT_WITHIN(
        0.001f,
        expectedPitch,
        state.pitch);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.0f,
        state.yaw);
}

void test_non_positive_dt_does_not_integrate()
{
    AttitudeEstimator estimator(0.5f);

    ImuSample sample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 1.0f,
        .gyroZ = 2.0f
    };

    // First sample initializes pitch from the accelerometer.
    estimator.update(sample, 0.01f);

    const AttitudeState zeroDt =
        estimator.update(sample, 0.0f);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, zeroDt.pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, zeroDt.yaw);

    const AttitudeState negativeDt =
        estimator.update(sample, -0.01f);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, negativeDt.pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, negativeDt.yaw);
}

void test_zero_acceleration_uses_gyro_only()
{
    AttitudeEstimator estimator(0.5f);

    ImuSample initialSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    // Initialize at zero pitch.
    estimator.update(initialSample, 0.01f);

    ImuSample zeroAccelSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 0.0f,
        .gyroX = 0.0f,
        .gyroY = 1.0f,
        .gyroZ = 0.0f
    };

    const AttitudeState state =
        estimator.update(zeroAccelSample, 0.1f);

    // With no valid gravity direction, pitch must come only
    // from gyro integration: 0 + 1 rad/s * 0.1 s = 0.1 rad.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.1f,
        state.pitch);
}

void test_zero_acceleration_does_not_initialize_estimator()
{
    AttitudeEstimator estimator(0.5f);

    ImuSample invalidSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 0.0f,
        .gyroX = 0.0f,
        .gyroY = 1.0f,
        .gyroZ = 2.0f
    };

    const AttitudeState invalidState =
        estimator.update(invalidSample, 0.1f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, invalidState.pitch);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, invalidState.yaw);

    ImuSample validSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 1.0f,
        .gyroZ = 2.0f
    };

    const AttitudeState initializedState =
        estimator.update(validSample, 0.1f);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, initializedState.pitch);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.0f, initializedState.yaw);
}

void test_large_acceleration_uses_gyro_only()
{
    AttitudeEstimator estimator(0.5f);

    ImuSample initialSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    // Initialize at zero pitch with a valid 1 g sample.
    estimator.update(initialSample, 0.01f);

    ImuSample dynamicSample{
        .accelX = 9.80665f,
        .accelY = 0.0f,
        .accelZ = 2.0f * 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 1.0f,
        .gyroZ = 0.0f
    };

    const AttitudeState state =
        estimator.update(dynamicSample, 0.1f);

    // Dynamic acceleration is outside the trusted gravity range.
    // Use gyro only: 0 + 1 rad/s * 0.1 s = 0.1 rad.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.1f,
        state.pitch);
}

void test_low_acceleration_uses_gyro_only()
{
    AttitudeEstimator estimator(0.5f);

    ImuSample initialSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    // Initialize at zero pitch with a valid 1 g sample.
    estimator.update(initialSample, 0.01f);

    ImuSample dynamicSample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 0.5f * 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 1.0f,
        .gyroZ = 0.0f
    };

    const AttitudeState state =
        estimator.update(dynamicSample, 0.1f);

    // Dynamic acceleration is outside the trusted gravity range.
    // Use gyro only: 0 + 1 rad/s * 0.1 s = 0.1 rad.
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.1f,
        state.pitch);
}

void test_tilted_one_g_acceleration_is_trusted()
{
    AttitudeEstimator estimator(0.5f);

    constexpr float g = 9.80665f;

    ImuSample sample{
        .accelX = 0.5f * g,       // sin(30°)
        .accelY = 0.0f,
        .accelZ = 0.8660254f * g, // cos(30°)
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    const AttitudeState state =
        estimator.update(sample, 0.01f);

    constexpr float expectedPitch =
        0.5235988f; // 30° in radians

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        expectedPitch,
        state.pitch);
}

void test_estimator_is_not_initialized_at_startup()
{
    AttitudeEstimator estimator(0.5f);

    TEST_ASSERT_FALSE(estimator.initialized());
}

void test_untrusted_acceleration_does_not_initialize_estimator()
{
    AttitudeEstimator estimator(0.5f);

    ImuSample sample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 0.5f * 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    estimator.update(sample, 0.001f);

    TEST_ASSERT_FALSE(estimator.initialized());
}

void test_trusted_acceleration_initializes_estimator()
{
    AttitudeEstimator estimator(0.5f);

    ImuSample sample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    estimator.update(sample, 0.001f);

    TEST_ASSERT_TRUE(estimator.initialized());
}

void test_reset_clears_initialized_state()
{
    AttitudeEstimator estimator(0.5f);

    ImuSample sample{
        .accelX = 0.0f,
        .accelY = 0.0f,
        .accelZ = 9.80665f,
        .gyroX = 0.0f,
        .gyroY = 0.0f,
        .gyroZ = 0.0f
    };

    estimator.update(sample, 0.001f);

    TEST_ASSERT_TRUE(estimator.initialized());

    estimator.reset();

    TEST_ASSERT_FALSE(estimator.initialized());
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_stationary_upright_robot_has_zero_attitude);
    RUN_TEST(test_initial_pitch_is_estimated_from_accelerometer);
    RUN_TEST(test_gyro_contributes_to_pitch_estimate);
    RUN_TEST(test_accelerometer_corrects_pitch_drift);
    RUN_TEST(test_filter_behavior_is_consistent_across_sample_rates);
    RUN_TEST(test_gyro_is_integrated_into_yaw);
    RUN_TEST(test_reset_reinitializes_attitude);
    RUN_TEST(test_non_positive_dt_does_not_integrate);
    RUN_TEST(test_zero_acceleration_uses_gyro_only);
    RUN_TEST(test_zero_acceleration_does_not_initialize_estimator);
    RUN_TEST(test_large_acceleration_uses_gyro_only);
    RUN_TEST(test_low_acceleration_uses_gyro_only);
    RUN_TEST(test_tilted_one_g_acceleration_is_trusted);
    RUN_TEST(test_estimator_is_not_initialized_at_startup);
    RUN_TEST(test_untrusted_acceleration_does_not_initialize_estimator);
    RUN_TEST(test_trusted_acceleration_initializes_estimator);
    RUN_TEST(test_reset_clears_initialized_state);

    return UNITY_END();
}