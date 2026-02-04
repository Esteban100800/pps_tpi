#include "esp_web_server.h"
#include "mpu_dmp.h"
#include "PIDController.h"
#include "servo_control.h"
#include "car_functions.h"
#include "dubin.h"
#include "position.h"
#include "differential.h"
#include "car_constants.h"

HiWonderMotors car;

PIDController pid_motor_right(0.5, 2.0, 0.1); // 2.0 en kp reduce las oscilaciones iniciales, menor a 2.0 no es eficiente
PIDController pid_motor_left(0.5, 2.0, 0.1);
PIDController pid_servo_hiwonder(4.0, 6.0, 0.05);

MPUDMP mpu(2);

QueueHandle_t yawQueue;
QueueHandle_t motorQueue;  // Cola para enviar un int
QueueHandle_t motor2Queue; // Cola para enviar un int
QueueHandle_t positionQueue;
ElectronicDifferential electronicDiff(WHEEL_BASE, MAX_RPM, LENGTH, WHEEL_RADIUS, DELTA_MAX); // wheelBase, maxRPM, Length, wheelRadius, delta_max (30° en radianes)

PositionEstimator odom(WHEEL_RADIUS, WHEEL_BASE, car, pid_motor_right, pid_motor_left); // radio rueda, ancho

DubinsPlanner planner(PATH_RADIUS);
DubinsPath path;

ESPWebServer myServer(pid_servo_hiwonder, pid_motor_left, pid_motor_right, odom, planner); // motor, pid_servo, pid_motor

void controlTask(void *parameter)
{
    while (1)
    {
        if (myServer.newGoalAvailable() && !odom.move())
        {
            Pose pendingGoal = myServer.getPendingGoal();

            Pose start = {
                odom.getPosition().x,
                odom.getPosition().y,
                odom.getPosition().theta};

            planner.setStartPose(start);
            planner.setGoalPose(pendingGoal);

            planner.setSegmentIndex(0);
            planner.setNewSegment(true);

            odom.reset_segment_distance();
            odom.setFinished(false);
            odom.setMove(true);

            for (int i = 0; i < 3; i++)
                odom.segments_distances[i] = 0.0f;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void handleServer(void *parameter)
{
    float yawValue = 0;
    float motorValue = 0;
    float motor2Value = 0;

    while (1)
    {
        // Espera hasta 10ms por un valor nuevo (reducido de 50ms para respuesta más rápida)
        if (xQueueReceive(yawQueue, &yawValue, 10 / portTICK_PERIOD_MS) == pdTRUE)
        {
            myServer.updateSensor(yawValue);
        }
        if (xQueueReceive(motorQueue, &motorValue, 10 / portTICK_PERIOD_MS) == pdTRUE)
        {
            myServer.updateMotor(motorValue);
        }
        if (xQueueReceive(motor2Queue, &motor2Value, 10 / portTICK_PERIOD_MS) == pdTRUE)
        {
            myServer.updateMotor2(motor2Value);
        }

        myServer.loop();
        vTaskDelay(20 / portTICK_PERIOD_MS); // Reducido de 100ms a 20ms para respuesta más rápida
    }
}

int pulses;

float filteredRPM[4] = {0};

float getRPM_hiwonder(MotorID motorID)
{
    int pulses = abs(car.GetEncoderCount(motorID));
    float rpm = pulses * 60.0 / (ENCODER_PULSES_PER_REV * DT_RPM_MEASUREMENT); // 1320 pulsos por revolución, 0.01s intervalo

    filteredRPM[motorID] = ALPHA * filteredRPM[motorID] + BETHA * rpm + BETHA * filteredRPM[motorID];

    car.SetEncoderCount(motorID, 0);
    return filteredRPM[motorID];
}

static uint32_t lastPrint = 0;
float rpmRight, rpmLeft;

void change_speed_hiwonder(void *pvParameters)
{
    while (1)
    {
        if (myServer.isSpeedRequested())
        {
            float requestedSpeed = myServer.getRequestedSpeed();
            electronicDiff.computeWheelSpeeds(requestedSpeed);
            odom.setSpeed(requestedSpeed);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void car_control(void *pvParameters)
{
    if (!car.begin())
    {
        // Serial.println("❌ HiWonder no responde por I2C");
        vTaskDelete(NULL);
    }

    // Serial.println("✅ HiWonder inicializado");

    /* ===== SERVO ===== */
    const int SERVO_PIN = 4; // el pin que estés usando
    car.attachServo(SERVO_PIN);
    float anguloServo = 0.0; // posición actual

    /* ===== MOTORES ===== */
    const MotorID motorRight = MOTOR_1;
    const MotorID motorLeft = MOTOR_2;

    const TickType_t LOOP = pdMS_TO_TICKS(10);

    mpu.begin();

    electronicDiff.computeWheelSpeeds(0.13f);
    delay(1000);
    while (1)
    {

        if (odom.move())
        {
            Serial.println("\n=== DUBINS LRL TEST ===");

            if (!planner.compute(planner.getStartPose(), planner.getGoalPose(), path))
            {
                vTaskDelete(NULL);
            }

            for (int i = 0; i < 3; i++)
            {
                const char *typeStr =
                    (path.seg[i].type == SEG_LEFT) ? "LEFT" : (path.seg[i].type == SEG_RIGHT) ? "RIGHT"
                                                                                              : "STRAIGHT";
            }

            odom.setFinished(false);
            odom.setMove(false);
        }

        if (!odom.isFinished())
        {
            /* ========= SERVO ========= */
            float objetivoServo = 0.0;

            if (odom.getDireccion() == 1)
            {
                objetivoServo = 30.0;
                car.setServoAngle(objetivoServo);
            }
            else if (odom.getDireccion() == -1)
            {
                objetivoServo = -45.0;
                car.setServoAngle(objetivoServo);
            }
            else if (odom.getDireccion() == 0)
            {
                pid_servo_hiwonder.reset();
                pid_servo_hiwonder.setSetpoint(planner.getAngleStraight());
                float yaw_mpu = mpu.getYaw();
                float out_servo = pid_servo_hiwonder.compute(yaw_mpu, 30.0);
                out_servo = constrain(out_servo, -45.0, 30.0);
                objetivoServo = out_servo;
                car.setServoAngle(objetivoServo);
            }

            // movimiento suave (anti-latigazo)
            /*float delta = objetivoServo - anguloServo;
            delta = constrain(delta, -3.0, 3.0);
            anguloServo += delta;*/

            /* ========= MOTORES ========= */
            rpmRight = getRPM_hiwonder(motorRight);
            rpmLeft = getRPM_hiwonder(motorLeft);

            float outRight = pid_motor_right.compute(rpmRight, 50.0);
            float outLeft = pid_motor_left.compute(rpmLeft, 50.0);

            outRight = constrain(outRight, 0, 50);
            outLeft = constrain(outLeft, 0, 50);

            car.setMotorPWM(motorRight, (int8_t)outRight, SPEED_MODE);
            car.setMotorPWM(motorLeft, (int8_t)outLeft, SPEED_MODE);

            if (planner.getSegmentIndex() < 3)
            {
                if (path.seg[planner.getSegmentIndex()].type == SEG_LEFT)
                {
                    odom.setDireccion(1);

                    pid_motor_left.setSetpoint(electronicDiff.getDifferential().leftRPM);
                    pid_motor_right.setSetpoint(electronicDiff.getDifferential().rightRPM);
                }
                else if (path.seg[planner.getSegmentIndex()].type == SEG_RIGHT)
                {
                    odom.setDireccion(-1);
                    pid_motor_left.setSetpoint(electronicDiff.getDifferential().rightRPM);
                    pid_motor_right.setSetpoint(electronicDiff.getDifferential().leftRPM);
                }
                else
                {
                    odom.setDireccion(0);
                    float averageRPM = (electronicDiff.getDifferential().leftRPM + electronicDiff.getDifferential().rightRPM) / 2.0f;
                    pid_motor_right.setSetpoint(averageRPM);
                    pid_motor_left.setSetpoint(averageRPM);
                }
                float segment_distance = odom.getSegmentDistance() - odom.segments_distances[planner.getSegmentIndex()];
                if (segment_distance >= path.seg[planner.getSegmentIndex()].length)
                {
                    // Serial.printf("Segmento %d completado. Distancia: %.3f m\n", planner.getSegmentIndex() + 1, segment_distance);
                    odom.segments_distances[planner.getSegmentIndex()] += path.seg[planner.getSegmentIndex()].length;
                    odom.reset_segment_distance();
                    planner.setSegmentIndex(planner.getSegmentIndex() + 1);
                    planner.setNewSegment(true);

                    // Resetear PIDs al cambiar de segmento
                    pid_motor_left.reset();
                    pid_motor_right.reset();
                    pid_servo_hiwonder.reset();

                    if (planner.getSegmentIndex() >= 3)
                    {
                        odom.setFinished(true);
                        // Serial.println("🏁 Trayectoria completada");
                    }
                }
            }

            mpu.update();

            if (planner.getNewSegment())
            {
                if (path.seg[planner.getSegmentIndex()].type == SEG_STRAIGHT)
                {
                    planner.setAngleStraight(mpu.getYaw());
                }
                planner.setNewSegment(false);
            }

            vTaskDelay(LOOP);
        }
        else
        {
            // estado STOP
            car.stopAll();
            pid_motor_left.reset();
            pid_motor_right.reset();
            pid_servo_hiwonder.reset();
            car.setServoAngle(0.0); // centrar servo
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        // Solo actualizar odometría si hay movimiento real
        if (!odom.isFinished() && (abs(rpmRight) > 2.0f || abs(rpmLeft) > 2.0f))
        {
            odom.updateFromRPM(rpmRight, rpmLeft, 0.01f, mpu.getYaw() * PI / 180.0f);
        }

        Position p = odom.getPosition();

        uint32_t now = millis();
        if (now - lastPrint >= 130)
        {
            lastPrint = now;
            mpu.update();
            float yaw = mpu.getYaw();
            xQueueSend(yawQueue, &yaw, 0);
            xQueueSend(motorQueue, &rpmLeft, 0);
            xQueueSend(motor2Queue, &rpmRight, 0);
        }
    }
}

void setup()
{
    // Configurar Serial solo si es necesario, sin bloquear
    Serial.begin(115200);

    // Breve delay para estabilización, pero no dependiente de Serial
    delay(1000);

    // mpu.begin();
    myServer.begin();

    yawQueue = xQueueCreate(10, sizeof(float));
    // 10 elementos de tipo float
    if (yawQueue == NULL)
    {
        if (Serial)
            Serial.println("Error creando la cola");
        while (1)
            ;
    }
    motorQueue = xQueueCreate(10, sizeof(float)); // Cambiar de int a float

    if (motorQueue == NULL)
    {
        if (Serial)
            Serial.println("Error creando la cola del motor");
        while (1)
            ;
    }

    motor2Queue = xQueueCreate(10, sizeof(float)); // Cambiar de int a float
    if (motor2Queue == NULL)
    {
        if (Serial)
            Serial.println("Error creando la cola del motor2");
        while (1)
            ;
    }

    positionQueue = xQueueCreate(5, sizeof(Position));

    if (positionQueue == NULL)
    {
        if (Serial)
            Serial.println("Error creando la cola de posición");
        while (1)
            ;
    }

    xTaskCreatePinnedToCore(handleServer, "Server", 8192, NULL, 2, NULL, 0); // Core 1, prioridad alta
    // xTaskCreatePinnedToCore(TaskServoPID, "ServoPID", 4096, NULL, 1, NULL, 0);  // Core 0, prioridad media
    // xTaskCreatePinnedToCore(TaskPID, "MotorPID", 4096, NULL, 1, NULL, 0);       // Core 0, prioridad media
    // xTaskCreatePinnedToCore(TaskPID2, "Motor2PID", 4096, NULL, 1, NULL, 1);     // Core 1, prioridad media

    // xTaskCreatePinnedToCore(TaskMPUTest, "MPU_Test", 4096, NULL, 1, NULL, 0); // Core 0, prioridad media
    //  xTaskCreatePinnedToCore( TaskMotorPID_Test,"Motor_PID_Test",4096,NULL,1,NULL,0);
    // xTaskCreatePinnedToCore(TaskMotorPID, "Motor_PID_Left", 4096, NULL, 1, NULL, 0); //!!!!!!!!!!!!!!!!!!!!
    // xTaskCreatePinnedToCore( servo_Test,"servo_PID_Test",4096,NULL,1,NULL,0);
    // xTaskCreatePinnedToCore(dubinsTestTask, "DubinsTest", 4096, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(car_control, "CarControl", 8192, NULL, 1, NULL, 1);  // Core 1, prioridad media
    xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 1, NULL, 0); // Core 1, prioridad media

    xTaskCreatePinnedToCore(change_speed_hiwonder, "ChangeSpeed", 8192, NULL, 1, NULL, 0); // Aumentado de 4096 a 8192

    // xTaskCreatePinnedToCore(TaskSerialControl, "SerialControl", 2048, NULL, 1, NULL, 1); //!!!!!!!!!!!!!!!!!!!!!!
}

void loop()
{
    vTaskDelete(NULL);
}
