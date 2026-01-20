#include "esp_web_server.h"
#include "mpu_dmp.h"
#include "PIDController.h"
#include "servo_control.h"
#include "car_functions.h"
#include "dubin.h"
#include "position.h"
#include "differential.h"

HiWonderMotors car;

Motor motor(25, 26, 26, 3, 35);           // IA1, IA2, PWM pin, channel, Encoder pin
PIDController pid_servo(1.9, 10.0, 1e-4); // 2.0 en kp reduce las oscilaciones iniciales, menor a 2.0 no es eficiente
PIDController pid_motor(2.0, 2.0, 0);     // 2.0 en kp reduce las oscilaciones iniciales, menor a 2.0 no es eficiente

PIDController pid_motor_right(0.6, 2.0, 0.2); // 2.0 en kp reduce las oscilaciones iniciales, menor a 2.0 no es eficiente
PIDController pid_motor_left(0.6, 2.0, 0.2);
PIDController pid_servo_hiwonder(4.0, 6.0, 0.05);
Motor motor2(32, 33, 33, 4, 12);
PIDController pid_motor2(2.0, 4.0, 0); // PID separado para el segundo motor
// Declarar el servidor después de los objetos que necesita

MPUDMP mpu(2);

ServoControl myServo(27); // Pin del servo: 27

QueueHandle_t yawQueue;
QueueHandle_t motorQueue;  // Cola para enviar un int
QueueHandle_t motor2Queue; // Cola para enviar un int

ElectronicDifferential electronicDiff(0.18f, 100.0f, 0.175f, 0.035f, 0.5236f); // wheelBase, maxRPM, Length, wheelRadius, delta_max (30° en radianes)

PositionEstimator odom(0.035f, 0.18f, car, pid_motor_right, pid_motor_left); // radio rueda, ancho

DubinsPlanner planner(0.345f);

DubinsPath path;

ESPWebServer myServer(motor, pid_servo, pid_motor, motor2, pid_motor2, odom, planner); // motor, pid_servo, pid_motor

float yaw = 123.45;
volatile bool started = false;

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

void TaskServoPID(void *pvParameters)
{

    float delta_alpha_factor = 180 / 88.00; // relacion angulo_Servo/angulo_real_mpu(42.0° a  -46°)
    // medido con el MPU a 42° y el servo a 0° (0° en el servo es 48° en el MPU con la suma de 90º)
    myServo.begin();
    delay(1000);
    myServo.move();
    delay(1000);
    mpu.begin();

    if (Serial)
        Serial.println("✅ MPU, Motor, Servo y PID inicializados");
    pid_servo.setSetpoint(55.0); // Setpoint inicial para el servo

    while (1)
    {
        mpu.update();
        float yaw = (90 - mpu.getYaw());
        if (yaw > 136)
            yaw = 136;
        if (yaw < 48)
            yaw = 48; // Este es el valor mínimo que se considera para el servo, ya que medimos desde los 0 a 42 en el MPU
        yaw = ((yaw * (delta_alpha_factor)) - (48 * (delta_alpha_factor)));
        float output = pid_servo.compute(yaw, 180.0);
        output = yaw + output;
        output > 180 ? output = 180 : output < 0 ? output = 0
                                                 : output;
        myServo.write((int)output);

        xQueueSend(yawQueue, &yaw, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(20)); // Cada 20 ms
    }
}

void TaskPID(void *pvParameters)
{
    float lastPrintTime = 0; // Último tiempo de impresión
    motor.begin();
    delay(1000);
    if (Serial)
        Serial.println("✅ Motor y PID inicializados");
    pid_motor.setSetpoint(12.0); // Setpoint inicial para el motor

    while (1)
    {
        float rpm = motor.getRPM();
        motor.resetEncoder();
        float output = pid_motor.compute(rpm, 255.0); // en RPM
        // float pwm = map (output, minRPM, maxRPM, 0, 255); // Convertir a PWM (0-255)
        output > 255 ? output = 255 : output < 0 ? output = 0
                                                 : output;
        motor.setPWM((int)output);

        unsigned long now = millis();
        if (now - lastPrintTime >= 50) // Reducido de 100ms a 50ms para actualización más rápida
        {
            lastPrintTime = now;

            // Enviar RPM a la cola para el servidor web
            xQueueSend(motorQueue, &rpm, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Cada 20 ms
    }
}

void TaskPID2(void *pvParameters)
{
    float lastPrintTime = 0; // Último tiempo de impresión
    motor2.begin();
    delay(1000);
    if (Serial)
        Serial.println("✅ Motor2 y PID2 inicializados");
    pid_motor2.setSetpoint(12.0); // Setpoint inicial para el motor2

    while (1)
    {
        float rpm = motor2.getRPM();
        motor2.resetEncoder();
        float output = pid_motor2.compute(rpm, 255.0); // Usar pid_motor2 en lugar de pid_motor
        // float pwm = map (output, minRPM, maxRPM, 0, 255); // Convertir a PWM (0-255)
        output > 255 ? output = 255 : output < 0 ? output = 0
                                                 : output;
        motor2.setPWM((int)output);

        unsigned long now = millis();
        if (now - lastPrintTime >= 50) // Reducido de 100ms a 50ms para actualización más rápida
        {
            lastPrintTime = now;

            // Enviar RPM a la cola para el servidor web
            xQueueSend(motor2Queue, &rpm, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Cada 20 ms
    }
}

void TaskMPUTest(void *pvParameters)
{
    mpu.begin();
    delay(1000);

    if (Serial)
        Serial.println("🧭 MPU6050 DMP inicializada (modo test)");

    while (1)
    {
        mpu.update();

        float yaw = mpu.getYaw();        // yaw acumulado
        float yaw_raw = mpu.getRawYaw(); // yaw crudo del DMP
        float err = mpu.getAccumulatedError();

        Serial.printf(
            "Yaw: %7.2f° | Raw: %7.2f° | Err acum: %7.4f\n",
            yaw, yaw_raw, err);

        vTaskDelay(pdMS_TO_TICKS(150)); // 20 Hz
    }
}

void mpuTimerCallback(TimerHandle_t xTimer)
{
    mpu.update();

    float yaw = mpu.getYaw();
    float yaw_raw = mpu.getRawYaw();
    float err = mpu.getAccumulatedError();

    Serial.printf(
        "[MPU TIMER] Yaw: %7.2f | Raw: %7.2f | Err: %7.4f\n",
        yaw, yaw_raw, err);
}
int pulses;

float filteredRPM[4] = {0};

float getRPM_hiwonder(MotorID motorID)
{
    int pulses = abs(car.GetEncoderCount(motorID));
    float rpm = pulses * 60.0 / (1320.0 * 0.01); // 1320 pulsos por revolución, 0.01s intervalo

    filteredRPM[motorID] = 0.85 * filteredRPM[motorID] + 0.0728 * rpm + 0.0728 * filteredRPM[motorID];

    car.SetEncoderCount(motorID, 0);
    return filteredRPM[motorID];
}

volatile int direccion = -1; // 1 der, -1 izq, 0 recto

static uint32_t lastPrint = 0;

void TaskMotorPID(void *pvParameters)
{
    if (!car.begin())
    {
        Serial.println("❌ HiWonder no responde por I2C");
        vTaskDelete(NULL);
    }

    Serial.println("✅ HiWonder inicializado");

    /* ===== SERVO ===== */
    const int SERVO_PIN = 4; // el pin que estés usando
    car.attachServo(SERVO_PIN);
    float anguloServo = 0.0; // posición actual

    /* ===== MOTORES ===== */
    const MotorID motorRight = MOTOR_1;
    const MotorID motorLeft = MOTOR_2;

    const TickType_t LOOP = pdMS_TO_TICKS(10);

    mpu.begin();
    delay(1000);
    while (1)
    {

        if (started)
        {
            /* ========= SERVO ========= */
            float objetivoServo = 0.0;

            if (direccion == 1)
            {
                objetivoServo = 30.0;
                car.setAngle(objetivoServo);
            }
            else if (direccion == -1)
            {
                objetivoServo = -45.0;
                car.setAngle(objetivoServo);
            }
            else if (direccion == 0)
            {
                objetivoServo = 0.0;
                car.setAngle(objetivoServo);
            }

            // movimiento suave (anti-latigazo)
            float delta = objetivoServo - anguloServo;
            delta = constrain(delta, -3.0, 3.0);
            anguloServo += delta;

            car.setServoAngle(anguloServo);

            /* ========= MOTORES ========= */
            float rpmRight = getRPM_hiwonder(motorRight);
            float rpmLeft = getRPM_hiwonder(motorLeft);

            float outRight = pid_motor_right.compute(rpmRight, 50.0);
            float outLeft = pid_motor_left.compute(rpmLeft, 50.0);

            outRight = constrain(outRight, 0, 50);
            outLeft = constrain(outLeft, 0, 50);

            car.setMotorPWM(motorRight, (int8_t)outRight, SPEED_MODE);
            car.setMotorPWM(motorLeft, (int8_t)outLeft, SPEED_MODE);

            /* ========= DEBUG =========
          Serial.printf(
                "DIR=%d | Servo=%.1f | R RPM=%.1f PWM=%.1f | L RPM=%.1f PWM=%.1f\n",s
                direccion, anguloServo,
                rpmRight, outRight,
                rpmLeft, outLeft); */

            /* ========= ODOM + MPU (después de 100 loops) ========= */
            mpu.update();
            odom.updateFromRPM(rpmRight, rpmLeft, 0.01f, mpu.getYaw() * PI / 180.0f); // dt=10ms

            Position p = odom.getPosition();

            uint32_t now = millis();
            if (now - lastPrint >= 150)
            {
                lastPrint = now;

                Serial.printf(
                    "[ODOM] x=%.3f y=%.3f theta=%.1f°\n total distance: %.3f m | total distance integral: %.3f m\n",
                    p.x, p.y, p.theta * 180.0f / PI, odom.getDistance(), odom.getDistanceIntegral());

                Serial.printf(
                    "[MPU] Yaw: %7.2f | Raw: %7.2f | Err: %7.4f\n",
                    mpu.getYaw(),
                    mpu.getRawYaw(),
                    mpu.getAccumulatedError());
            }
            vTaskDelay(LOOP);
        }
        else
        {
            // estado STOP
            car.stopAll();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void car_control(void *pvParameters)
{
    if (!car.begin())
    {
        Serial.println("❌ HiWonder no responde por I2C");
        vTaskDelete(NULL);
    }

    Serial.println("✅ HiWonder inicializado");

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
                Serial.println("❌ No existe trayectoria");

                vTaskDelete(NULL);
            }

            Serial.println("✅ Trayectoria válida");
            Serial.printf("Longitud total: %.3f m\n", path.totalLength);

            for (int i = 0; i < 3; i++)
            {
                const char *typeStr =
                    (path.seg[i].type == SEG_LEFT) ? "LEFT" : (path.seg[i].type == SEG_RIGHT) ? "RIGHT"
                                                                                              : "STRAIGHT";

                Serial.printf(
                    "Segmento %d: %s | L = %.3f m\n",
                    i + 1,
                    typeStr,
                    path.seg[i].length);
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
            float rpmRight = getRPM_hiwonder(motorRight);
            float rpmLeft = getRPM_hiwonder(motorLeft);

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
                    Serial.printf("Segmento %d completado. Distancia: %.3f m\n", planner.getSegmentIndex() + 1, segment_distance);
                    odom.segments_distances[planner.getSegmentIndex()] += path.seg[planner.getSegmentIndex()].length;
                    odom.reset_segment_distance();
                    planner.setSegmentIndex(planner.getSegmentIndex() + 1);
                    planner.setNewSegment(true);

                    if (planner.getSegmentIndex() >= 3)
                    {
                        odom.setFinished(true);
                        Serial.println("🏁 Trayectoria completada");
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
            odom.updateFromRPM(rpmRight, rpmLeft, 0.01f, mpu.getYaw() * PI / 180.0f); // dt=10ms

            Position p = odom.getPosition();

            uint32_t now = millis();
            if (now - lastPrint >= 150)
            {
                lastPrint = now;

                Serial.printf(
                    "[ODOM] x=%.3f y=%.3f theta=%.1f°\n total distance: %.3f m | total distance integral: %.3f m\n",
                    p.x, p.y, p.theta * 180.0f / PI, odom.getDistance(), odom.getDistanceIntegral());

                Serial.printf(
                    "[MPU] Yaw: %7.2f | Raw: %7.2f | Err: %7.4f\n",
                    mpu.getYaw(),
                    mpu.getRawYaw(),
                    mpu.getAccumulatedError());
            }
            vTaskDelay(LOOP);
        }
        else
        {
            // estado STOP
            car.stopAll();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void TaskSerialControl(void *pvParameters)
{
    while (1)
    {
        if (Serial.available() > 0)
        {
            char c = Serial.read();

            if (c == 's' || c == 'S')
            {
                started = true;
                Serial.println("▶ STARTED = 1");
                odom.setMove(true);

                planner.compute_rmin(34);
                Pose goal = {1, 1, M_PI / 2};
                Pose start= {0, 0, 0};
                planner.setStartPose(start);
                planner.setGoalPose(goal);

                /*electronicDiff.computeWheelSpeeds(0.13f);

                ElectronicDifferential::differential diff = electronicDiff.getDifferential();

                Serial.printf("Right RPM: %.2f | Left RPM: %.2f\n", diff.rightRPM, diff.leftRPM);

                pid_motor_left.setSetpoint(diff.rightRPM);
                pid_motor_right.setSetpoint(diff.leftRPM);*/
            }
            else if (c == 'p' || c == 'P')
            {
                if (!odom.move())
                {
                    Pose start = {1, 1, M_PI / 2};
                    Pose goal = {0, 0, 0};

                    planner.setStartPose(start);
                    planner.setGoalPose(goal);
                    planner.setSegmentIndex(0);
                    planner.setNewSegment(true);

                    odom.reset_segment_distance();

                    for (int i = 0; i < 3; i++)
                        odom.segments_distances[i] = 0.0f;

                    odom.setDireccion(0);
                    odom.setFinished(false);
                    odom.setMove(true);

                    Serial.println("🔁 Nuevo path iniciado");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // anti-spam + no bloquear CPU
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

    Serial.println("\n✅ Conectado a WiFi");
    Serial.print("📡 IP del ESP32: ");
    Serial.println(WiFi.softAPIP());

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

    xTaskCreatePinnedToCore(handleServer, "Server", 8192, NULL, 2, NULL, 1); // Core 1, prioridad alta
    // xTaskCreatePinnedToCore(TaskServoPID, "ServoPID", 4096, NULL, 1, NULL, 0);  // Core 0, prioridad media
    // xTaskCreatePinnedToCore(TaskPID, "MotorPID", 4096, NULL, 1, NULL, 0);       // Core 0, prioridad media
    // xTaskCreatePinnedToCore(TaskPID2, "Motor2PID", 4096, NULL, 1, NULL, 1);     // Core 1, prioridad media

    // xTaskCreatePinnedToCore(TaskMPUTest, "MPU_Test", 4096, NULL, 1, NULL, 0); // Core 0, prioridad media
    //  xTaskCreatePinnedToCore( TaskMotorPID_Test,"Motor_PID_Test",4096,NULL,1,NULL,0);
    // xTaskCreatePinnedToCore(TaskMotorPID, "Motor_PID_Left", 4096, NULL, 1, NULL, 0); //!!!!!!!!!!!!!!!!!!!!
    // xTaskCreatePinnedToCore( servo_Test,"servo_PID_Test",4096,NULL,1,NULL,0);
    // xTaskCreatePinnedToCore(dubinsTestTask, "DubinsTest", 4096, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(car_control, "CarControl", 8192, NULL, 1, NULL, 1); // Core 1, prioridad media

    xTaskCreatePinnedToCore(TaskSerialControl, "SerialControl", 2048, NULL, 1, NULL, 1); //!!!!!!!!!!!!!!!!!!!!!!
}

void loop()
{
    vTaskDelete(NULL);
}
