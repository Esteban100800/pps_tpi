#include "esp_web_server.h"
#include "mpu_dmp.h"
#include "PIDController.h"
#include "car_functions.h"
#include "dubin.h"
#include "reeds_shepp.h"
#include "position.h"
#include "differential.h"
#include "car_constants.h"
#include <mutex>

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

QueueHandle_t goalQueue;

// ----reed shepp----
ReedsShepp rs_planner(PATH_RADIUS);

std::vector<RSPathSegment> rs_path; // path calculado
volatile int rs_segment_index = 0;  // segmento actual
volatile bool rs_new_segment = false;
volatile bool rs_move = false;
volatile bool rs_finished = true;
volatile float rs_angle_straight = 0.0f;
float rs_seg_dist = 0.0f;               // distancia acumulada en el segmento actual
volatile float rs_arc_start_yaw = 0.0f; // yaw al inicio de cada segmento de arco
Pose rs_goal_pose = {0.0, 0.0, 0.0};    // goal actual, para resetear odom al terminar

// Flag que indica que el hardware (I2C, MPU, servo) ya fue inicializado
volatile bool hardwareReady = false;

// Filtro exponencial de RPM (compartido por car_control y rs_control)
float filteredRPM[4] = {0};

void SetPendingGoal(void *parameter)
{
    Pose newGoal;

    while (1)
    {
        if (myServer.getImmediateRequested())
        {
            newGoal = myServer.getImmediateGoal();

            // Aborta el trayecto en curso y descarta lo que hubiera en la cola
            xQueueReset(goalQueue);
            car.stopAll();
            pid_motor_left.reset();
            pid_motor_right.reset();
            pid_servo_hiwonder.reset();

            if (myServer.isDubins())
            {
                odom.setFinished(true);
                odom.setMove(false);
            }
            else
            {
                rs_move = false;
                rs_finished = true;
            }

            xQueueSend(goalQueue, &newGoal, 0);
            myServer.setImmediateRequested();
        }
        else if (myServer.getRequested())
        {
            newGoal = myServer.getPendingGoal();
            xQueueSend(goalQueue, &newGoal, 0);
            myServer.setRequested();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void controlTask(void *parameter)
{
    Pose nextGoal;

    while (1)
    {
        // Solo actua si el modo activo es Dubins y no hay un trayecto en curso
        if (myServer.isDubins() && !odom.move() && odom.isFinished())
        {
            if (xQueueReceive(goalQueue, &nextGoal, 0) == pdTRUE)
            {
                Pose start = {
                    odom.getPosition().x,
                    odom.getPosition().y,
                    odom.getPosition().theta};

                planner.setStartPose(start);
                planner.setGoalPose(nextGoal);

                planner.setSegmentIndex(0);
                planner.setNewSegment(true);

                odom.reset_segment_distance();
                odom.setFinished(false);
                odom.setMove(true);

                for (int i = 0; i < 3; i++)
                    odom.segments_distances[i] = 0.0f;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void controlTaskRS(void *parameter)
{
    Pose nextGoal;

    while (1)
    {
        // Solo actúa si el modo activo es Reeds-Shepp y no hay movimiento en curso
        if (myServer.isReedsShepp() && !rs_move)
        {
            if (xQueueReceive(goalQueue, &nextGoal, 0) == pdTRUE)
            {
                Position pos = odom.getPosition();
                Pose start = {pos.x, pos.y, pos.theta};

                Pose normStart = rs_planner.normalizePose(start);
                Pose normGoal = rs_planner.normalizePose(nextGoal);

                auto all_paths = rs_planner.getAllPaths(normStart, normGoal);
                int idx = rs_planner.getOptimalPathIndex(all_paths);

                if (idx >= 0)
                {
                    rs_path = all_paths[idx];
                    rs_planner.denormalizePath(rs_path);

                    // Guardar el goal para resetear la odometriaa al terminar
                    rs_goal_pose = nextGoal;

                    // Limpiar filtro de RPM siempre al iniciar un path nuevo
                    filteredRPM[MOTOR_1] = 0.0f;
                    filteredRPM[MOTOR_2] = 0.0f;
                    car.SetEncoderCount(MOTOR_1, 0);
                    car.SetEncoderCount(MOTOR_2, 0);

                    rs_segment_index = 0;
                    rs_seg_dist = 0.0f;
                    rs_new_segment = true;
                    rs_finished = false;

                    pid_motor_left.reset();
                    pid_motor_right.reset();
                    pid_servo_hiwonder.reset();

                    // Pausa para que el servo llegue a la posición inicial antes de arrancar
                    vTaskDelay(pdMS_TO_TICKS(200));

                    rs_move = true;
                }
                else
                {
                }
            }
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
        // Espera hasta 20ms por un valor nuevo
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
        vTaskDelay(20 / portTICK_PERIOD_MS); // Reducido de 100ms a 20ms para respuesta mas rapida
    }
}

int pulses;

float getRPM_hiwonder(MotorID motorID)
{
    int pulses = abs(car.GetEncoderCount(motorID));
    float rpm = pulses * 60.0f / (ENCODER_PULSES_PER_REV * DT_RPM_MEASUREMENT);

    // Descarta lecturas físicamente imposibles (encoder roto o inversión de marcha)
    if (rpm > MAX_RPM * 1.5f)
        rpm = filteredRPM[motorID];

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
            pid_motor_right.reset();
            pid_motor_left.reset();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void car_control(void *pvParameters)
{
    if (!car.begin())
    {
        // Serial.println(" HiWonder no responde por I2C");
        vTaskDelete(NULL);
    }

    // Serial.println(" HiWonder inicializado");

    /* ===== SERVO ===== */
    const int SERVO_PIN = 4; // el pin que estes usando
    car.attachServo(SERVO_PIN);
    float anguloServo = 0.0; // posicion actual

    /* ===== MOTORES ===== */
    const MotorID motorRight = MOTOR_1;
    const MotorID motorLeft = MOTOR_2;

    const TickType_t LOOP = pdMS_TO_TICKS(10);

    mpu.begin();

    electronicDiff.computeWheelSpeeds(0.13f);
    hardwareReady = true; // avisa a rs_control que puede usar el hardware

    delay(1000);
    while (1)
    {
        // Si el modo activo es Reeds-Shepp, esta tarea cede el control
        if (!myServer.isDubins())
        {
            vTaskDelay(LOOP);
            continue;
        }

        if (odom.move())
        {
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
        // Solo actualizar odometria si hay movimiento real
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


//  Tarea de control con Reeds-Shepp
//  Espeja la logica de car_control pero usa rs_path (segmentos variables,
//  incluye gear FORWARD / BACKWARD para marcha atrás).

void rs_control(void *pvParameters)
{
    // Espera a que car_control haya inicializado el hardware
    while (!hardwareReady)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    const MotorID motorRight = MOTOR_1;
    const MotorID motorLeft = MOTOR_2;
    const TickType_t LOOP = pdMS_TO_TICKS(10);

    while (1)
    {
        // Si el modo activo es Dubins, esta tarea cede el control
        if (!myServer.isReedsShepp())
        {
            vTaskDelay(LOOP);
            continue;
        }

        if (rs_move && !rs_finished)
        {
            if (rs_segment_index < (int)rs_path.size())
            {
                // Al entrar en un nuevo segmento: capturar yaw de referencia
                // ANTES de evaluar segmentDone (si no, se usa un yaw viejo,
                // sobrante del segmento/path anterior, y el segmento se salta).
                if (rs_new_segment)
                {
                    if (rs_path[rs_segment_index].type == STRAIGHT)
                        rs_angle_straight = mpu.getYaw(); // yaw objetivo para PID de servo
                    else
                        rs_arc_start_yaw = mpu.getYaw(); // yaw inicial del arco

                    rs_new_segment = false;
                }

                RSSegType seg_type = rs_path[rs_segment_index].type;
                Gear seg_gear = rs_path[rs_segment_index].gear;
                float motorDir = (seg_gear == FORWARD) ? 1.0f : -1.0f;

                // Servomotor: posición según tipo de segmento
                if (seg_type == LEFT)
                {
                    car.setServoAngle(30.0f);
                }
                else if (seg_type == RIGHT)
                {
                    car.setServoAngle(-45.0f);
                }
                else // STRAIGHT
                {
                    pid_servo_hiwonder.setSetpoint(rs_angle_straight);
                    float yaw_mpu = mpu.getYaw();
                    float out_servo = pid_servo_hiwonder.compute(yaw_mpu, 30.0f);
                    out_servo = constrain(out_servo, -45.0f, 30.0f);
                    // En marcha atrás la corrección del PID también se invierte
                    if (seg_gear == BACKWARD)
                        out_servo = -out_servo;
                    car.setServoAngle(out_servo);
                }

                /*  SETPOINTS DE RPM  */
                float sp_left = 0, sp_right = 0;
                if (seg_type == LEFT)
                {
                    sp_left = electronicDiff.getDifferential().leftRPM;
                    sp_right = electronicDiff.getDifferential().rightRPM;
                }
                else if (seg_type == RIGHT)
                {
                    sp_left = electronicDiff.getDifferential().rightRPM;
                    sp_right = electronicDiff.getDifferential().leftRPM;
                }
                else
                {
                    sp_left = sp_right = (electronicDiff.getDifferential().leftRPM +
                                          electronicDiff.getDifferential().rightRPM) /
                                         2.0f;
                }
                pid_motor_left.setSetpoint(sp_left);
                pid_motor_right.setSetpoint(sp_right);

                /*   MOTORES  */
                rpmRight = getRPM_hiwonder(motorRight);
                rpmLeft = getRPM_hiwonder(motorLeft);

                float outRight = pid_motor_right.compute(rpmRight, 50.0f);
                float outLeft = pid_motor_left.compute(rpmLeft, 50.0f);

                // constrain en positivo y luego aplicar dirección (FWD/BWD)
                outRight = constrain(outRight, 0.0f, 50.0f) * motorDir;
                outLeft = constrain(outLeft, 0.0f, 50.0f) * motorDir;

                car.setMotorPWM(motorRight, (int8_t)outRight, SPEED_MODE);
                car.setMotorPWM(motorLeft, (int8_t)outLeft, SPEED_MODE);

                /* ===== ODOMETRÍA Y DISTANCIA ===== */
                if (fabsf(rpmRight) > 2.0f || fabsf(rpmLeft) > 2.0f)
                {
                    // Para la odometría pasamos RPM con signo según la marcha
                    odom.updateFromRPM(rpmRight * motorDir, rpmLeft * motorDir,
                                       0.01f, mpu.getYaw() * PI / 180.0f);

                    float v_center = (rpmRight + rpmLeft) / 2.0f *
                                     (2.0f * PI / 60.0f) * WHEEL_RADIUS;
                    rs_seg_dist += fabsf(v_center) * 0.01f;
                }

                /* ===== CONDICIÓN DE FIN DE SEGMENTO =====
                 * ARCO:   cierra el lazo con el yaw del IMU.
                 *         Ángulo objetivo = longitud / radio  [rad] → grados.
                 *         La distancia actúa solo de seguridad (1.4× el arco esperado).
                 * RECTA:  usa distancia de encoders (el heading no cambia).
                 */
                bool segmentDone = false;
                if (rs_path[rs_segment_index].type != STRAIGHT)
                {
                    float targetDeg = (rs_path[rs_segment_index].length / PATH_RADIUS) * (180.0f / PI);
                    float curYaw = mpu.getYaw();
                    float delta = curYaw - rs_arc_start_yaw;
                    // Normalizar a (-180, 180] para cruzar el ±180° sin problemas
                    while (delta > 180.0f)
                        delta -= 360.0f;
                    while (delta <= -180.0f)
                        delta += 360.0f;
                    float arcDone = fabsf(delta);

                    segmentDone = (arcDone >= targetDeg) || (rs_seg_dist >= rs_path[rs_segment_index].length * 1.4f);
                }
                else
                {
                    segmentDone = (rs_seg_dist >= rs_path[rs_segment_index].length);
                }

                /* ===== CAMBIO DE SEGMENTO ===== */
                if (segmentDone)
                {
                    // Determinar si hay cambio de gear antes de avanzar el índice
                    bool gearChange = (rs_segment_index + 1 < (int)rs_path.size()) &&
                                      (rs_path[rs_segment_index].gear != rs_path[rs_segment_index + 1].gear);

                    rs_seg_dist = 0.0f;
                    rs_segment_index++;
                    rs_new_segment = true;

                    pid_motor_left.reset();
                    pid_motor_right.reset();
                    pid_servo_hiwonder.reset();

                    // Detener siempre al cambiar de segmento para que el servo
                    // llegue a su nueva posición antes de volver a moverse.
                    car.stopAll();
                    filteredRPM[MOTOR_1] = 0.0f;
                    filteredRPM[MOTOR_2] = 0.0f;
                    car.SetEncoderCount(MOTOR_1, 0);
                    car.SetEncoderCount(MOTOR_2, 0);

                    // El servo necesita tiempo para girar físicamente.
                    // Si además cambia de marcha (FWD↔BWD) se da más tiempo.
                    uint32_t pauseMs = gearChange ? 200 : 100;
                    vTaskDelay(pdMS_TO_TICKS(pauseMs));

                    if (rs_segment_index >= (int)rs_path.size())
                        rs_finished = true;
                }

                mpu.update();
            }
        }
        else if (rs_move && rs_finished)
        {
            // Trayectoria RS completada
            car.stopAll();
            car.setServoAngle(0.0f);
            pid_motor_left.reset();
            pid_motor_right.reset();
            pid_servo_hiwonder.reset();

            rs_move = false;
        }

        /* ===== TELEMETRÍA ===== */
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

        vTaskDelay(LOOP);
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
    motorQueue = xQueueCreate(10, sizeof(float)); 

    if (motorQueue == NULL)
    {
        if (Serial)
            Serial.println("Error creando la cola del motor");
        while (1)
            ;
    }

    motor2Queue = xQueueCreate(10, sizeof(float)); 
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

    goalQueue = xQueueCreate(10, sizeof(Pose)); // hasta 10 puntos

    if (goalQueue == NULL)
    {
        if (Serial)
            Serial.println("Error creando la cola de objetivos");
        while (1)
            ;
    }

    xTaskCreatePinnedToCore(handleServer, "Server", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(car_control, "CarControl", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(rs_control, "RSControl", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(controlTaskRS, "ControlTaskRS", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(change_speed_hiwonder, "ChangeSpeed", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(SetPendingGoal, "SetPendingGoal", 4096, NULL, 1, NULL, 0);
}

void loop()
{
    vTaskDelete(NULL);
}
