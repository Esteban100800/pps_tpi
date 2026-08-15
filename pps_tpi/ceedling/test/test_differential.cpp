#include "unity.h"
#include "differential.h"

static ElectronicDifferential *diff;

void setUp(void)
{
    // wheelBase=0.18m, maxRPM=1000 (alto para no enmascarar la formula), Length=0.175m, wheelRadius=0.035m, delta_max=0
    diff = new ElectronicDifferential(0.18f, 1000.0f, 0.175f, 0.035f, 0.0f);
}

void tearDown(void)
{
    delete diff;
}

void test_straight_line_both_wheels_equal(void)
{
    diff->computeWheelSpeeds(0.5f);
    ElectronicDifferential::differential d = diff->getDifferential();

    TEST_ASSERT_FLOAT_WITHIN(0.05f, 136.42f, d.leftRPM);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 136.42f, d.rightRPM);
}

void test_zero_speed_gives_zero_rpm(void)
{
    diff->computeWheelSpeeds(0.0f);
    ElectronicDifferential::differential d = diff->getDifferential();

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, d.leftRPM);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, d.rightRPM);
}

void test_turning_splits_wheel_speeds(void)
{
    diff->setdelta(0.5236f); // ~30 grados
    diff->computeWheelSpeeds(0.5f);
    ElectronicDifferential::differential d = diff->getDifferential();

    // Girando, la rueda exterior (derecha) debe ir mas rapido que la interior (izquierda)
    TEST_ASSERT_GREATER_THAN_FLOAT(d.leftRPM, d.rightRPM);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 176.94f, d.rightRPM);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 95.90f, d.leftRPM);
}

void test_rpm_is_clamped_to_max(void)
{
    diff->setmaxRPM(50.0f);
    diff->computeWheelSpeeds(10.0f); // velocidad exagerada para forzar el clamp

    ElectronicDifferential::differential d = diff->getDifferential();

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, d.leftRPM);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, d.rightRPM);
}

void test_rpm_is_clamped_to_negative_max(void)
{
    diff->setmaxRPM(50.0f);
    diff->computeWheelSpeeds(-10.0f);

    ElectronicDifferential::differential d = diff->getDifferential();

    TEST_ASSERT_FLOAT_WITHIN(0.001f, -50.0f, d.leftRPM);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -50.0f, d.rightRPM);
}

void test_setters_take_effect(void)
{
    diff->setwheelBase(0.20f);
    diff->setLength(0.20f);
    diff->setwheelRadius(0.04f);
    diff->setdelta(0.1f);
    diff->setmaxRPM(200.0f);

    // No hay getters de estos parametros: verificamos indirectamente que el
    // cambio de wheelRadius (0.035 -> 0.04) reduce la velocidad angular resultante.
    diff->computeWheelSpeeds(0.5f);
    ElectronicDifferential::differential d = diff->getDifferential();

    TEST_ASSERT_LESS_THAN_FLOAT(136.42f, d.rightRPM);
}
