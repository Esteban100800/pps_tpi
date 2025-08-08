#include "esp_web_server.h"
#include "mpu_dmp.h"
#include "PIDController.h"
#include "servo_control.h"

Motor motor(25, 26, 26, 3, 35);           // IA1, IA2, PWM pin, channel, Encoder pin
PIDController pid_servo(motor, 1.9, 10.0, 1e-4); // 2.0 en kp reduce las oscilaciones iniciales, menor a 2.0 no es eficiente
PIDController pid_motor(motor, 2.0,2.0,0); // 2.0 en kp reduce las oscilaciones iniciales, menor a 2.0 no es eficiente
Motor motor2(32, 33, 33, 4, 12); 
PIDController pid_motor2(motor2, 2.0, 2.0, 0); // PID separado para el segundo motor 
// Declarar el servidor después de los objetos que necesita
ESPWebServer myServer(motor, pid_servo, pid_motor, motor2, pid_motor2); // motor, pid_servo, pid_motor

MPUDMP mpu(2);

ServoControl myServo(27); // Pin del servo: 27

QueueHandle_t yawQueue;
QueueHandle_t motorQueue; // Cola para enviar un int
QueueHandle_t motor2Queue; // Cola para enviar un int

float yaw = 123.45;
bool started = false;

void handleServer(void *parameter)
{
    float yawValue = 0;
    float motorValue = 0;
    float motor2Value = 0;

    while (1)
    {
        // Espera hasta 50ms por un valor nuevo
        if (xQueueReceive(yawQueue, &yawValue, 50 / portTICK_PERIOD_MS) == pdTRUE) {
            myServer.updateSensor(yawValue);
        }
        if (xQueueReceive(motorQueue, &motorValue, 50 / portTICK_PERIOD_MS) == pdTRUE) {
            myServer.updateMotor(motorValue);
        }
        if (xQueueReceive(motor2Queue, &motor2Value, 50 / portTICK_PERIOD_MS) == pdTRUE) {
            myServer.updateMotor2(motor2Value);
        }

        myServer.loop();
        vTaskDelay(80 / portTICK_PERIOD_MS);
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
{   float lastPrintTime = 0; // Último tiempo de impresión  
    motor.begin();
    delay(1000);
    Serial.println("✅ Motor y PID inicializados");
    pid_motor.setSetpoint(12.0); // Setpoint inicial para el motor

    while (1)
    {
        float rpm = motor.getRPM();
        motor.resetEncoder();
        float output = pid_motor.compute(rpm, 255.0); // en RPM
        //float pwm = map (output, minRPM, maxRPM, 0, 255); // Convertir a PWM (0-255)
        output > 255 ? output = 255 : output < 0 ? output = 0
                                                 : output;
        motor.setPWM((int)output);

        unsigned long now = millis();
        if (now - lastPrintTime >= 100)
        {
            lastPrintTime = now;

            // Enviar RPM a la cola para el servidor web
            xQueueSend(motorQueue, &rpm, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Cada 20 ms
    }
}

void TaskPID2(void *pvParameters)
{   float lastPrintTime = 0; // Último tiempo de impresión
    motor2.begin();
    delay(1000);
    Serial.println("✅ Motor2 y PID2 inicializados");
    pid_motor2.setSetpoint(12.0); // Setpoint inicial para el motor2

    while (1)
    {
        float rpm = motor2.getRPM();
        motor2.resetEncoder();
        float output = pid_motor2.compute(rpm, 255.0); // Usar pid_motor2 en lugar de pid_motor
        //float pwm = map (output, minRPM, maxRPM, 0, 255); // Convertir a PWM (0-255)
        output > 255 ? output = 255 : output < 0 ? output = 0
                                                 : output;
        motor2.setPWM((int)output);

        unsigned long now = millis();
        if (now - lastPrintTime >= 100)
        {
            lastPrintTime = now;

            // Enviar RPM a la cola para el servidor web
            xQueueSend(motor2Queue, &rpm, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Cada 20 ms
    }
}

void setup()
{

    Serial.begin(115200);

    delay(1000);
    while (!Serial)
        ;

    Serial.println("Presioná una tecla para comenzar...");
    while (!Serial.available())
    {
    }
    Serial.read();

    //mpu.begin();
    myServer.begin();


    yawQueue = xQueueCreate(10, sizeof(float));
    // 10 elementos de tipo float
    if (yawQueue == NULL)
    {
        Serial.println("Error creando la cola");
        while (1)
            ;
    }
    motorQueue = xQueueCreate(10, sizeof(float)); // Cambiar de int a float

    if (motorQueue == NULL)
    {
        Serial.println("Error creando la cola del motor");
        while (1)
            ;
    }

    motor2Queue = xQueueCreate(10, sizeof(float)); // Cambiar de int a float
    if (motor2Queue == NULL)
    {
        Serial.println("Error creando la cola del motor2");
        while (1)
            ;
    }
    xTaskCreatePinnedToCore(handleServer, "Server", 8192, NULL, 2, NULL, 1);     // Core 1, prioridad alta
    xTaskCreatePinnedToCore(TaskServoPID, "ServoPID", 4096, NULL, 1, NULL, 0);  // Core 0, prioridad media
    xTaskCreatePinnedToCore(TaskPID, "MotorPID", 4096, NULL, 1, NULL, 0);       // Core 0, prioridad media
    xTaskCreatePinnedToCore(TaskPID2, "Motor2PID", 4096, NULL, 1, NULL, 1);     // Core 1, prioridad media
    started = true;
}

void loop()
{
}
