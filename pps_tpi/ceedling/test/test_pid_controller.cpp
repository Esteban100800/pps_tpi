#include "unity.h"
#include "PIDController.h"

static PIDController *pid;

void setUp(void)
{
    pid = new PIDController(2.0f, 0.0f, 0.0f);
}

void tearDown(void)
{
    delete pid;
}

void test_constructor_sets_gains(void)
{
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pid->getKp());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid->getKi());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid->getKd());
}

void test_setters_update_gains_and_setpoint(void)
{
    pid->setKp(1.5f);
    pid->setKi(0.5f);
    pid->setKd(0.25f);
    pid->setSetpoint(10.0f);

    TEST_ASSERT_EQUAL_FLOAT(1.5f, pid->getKp());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, pid->getKi());
    TEST_ASSERT_EQUAL_FLOAT(0.25f, pid->getKd());
    TEST_ASSERT_EQUAL_FLOAT(10.0f, pid->getSetpoint());
}

void test_pure_proportional_output(void)
{
    // ki = kd = 0 aisla el termino proporcional: output = kp * error
    pid->setSetpoint(10.0f);
    float output = pid->compute(4.0f, 1000.0f);

    TEST_ASSERT_EQUAL_FLOAT(12.0f, output); // kp(2) * error(6)
    TEST_ASSERT_EQUAL_FLOAT(6.0f, pid->getError());
}

void test_integral_accumulates_and_clamps(void)
{
    PIDController integral_pid(0.0f, 1.0f, 0.0f); // solo termino integral
    integral_pid.setSetpoint(10.0f);

    // error=10 en cada paso, dt interno=0.02s => integral += 0.2 por llamada
    integral_pid.compute(0.0f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, integral_pid.getIntegral());

    integral_pid.compute(0.0f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, integral_pid.getIntegral());

    // El tercer paso llevaria la integral a 0.6, pero el limite es 0.5 (anti-windup)
    float output = integral_pid.compute(0.0f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, integral_pid.getIntegral());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, output);
}

void test_integral_clamps_negative(void)
{
    PIDController integral_pid(0.0f, 1.0f, 0.0f);
    integral_pid.setSetpoint(-10.0f);

    integral_pid.compute(0.0f, 0.5f);
    integral_pid.compute(0.0f, 0.5f);
    integral_pid.compute(0.0f, 0.5f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, integral_pid.getIntegral());
}

void test_small_error_below_threshold_does_not_accumulate_integral(void)
{
    PIDController integral_pid(0.0f, 1.0f, 0.0f);
    integral_pid.setSetpoint(0.05f); // error = 0.05, por debajo del umbral delta_error (0.08)

    integral_pid.compute(0.0f, 10.0f);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, integral_pid.getIntegral());
}

void test_reset_clears_internal_state(void)
{
    pid->setSetpoint(10.0f);
    pid->compute(4.0f, 1000.0f);

    pid->reset();

    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid->getError());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid->getIntegral());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid->getRealDerivative());
}
