#include <unity.h>

#include "../../src/motor/MotorController.h"

class FakeMotorController : public MotorController
{
public:
    void apply(const ActuatorCommand& command) override
    {
        lastCommand = command;
        applyCount++;
    }

    ActuatorCommand lastCommand{
        .driveEnabled = false,
        .leftTorque = 0.0f,
        .rightTorque = 0.0f
    };

    int applyCount = 0;
};

void test_motor_controller_receives_actuator_command()
{
    FakeMotorController motorController;

    ActuatorCommand command{
        .driveEnabled = true,
        .leftTorque = 1.5f,
        .rightTorque = -2.0f
    };

    motorController.apply(command);

    TEST_ASSERT_EQUAL(1, motorController.applyCount);

    TEST_ASSERT_TRUE(
        motorController.lastCommand.driveEnabled);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        1.5f,
        motorController.lastCommand.leftTorque);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        -2.0f,
        motorController.lastCommand.rightTorque);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_motor_controller_receives_actuator_command);

    return UNITY_END();
}
