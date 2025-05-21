#include <Wire.h>
#include <mpu6500.h>
#include <ESP32Servo.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

#define ENA 32
#define ENB 33
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14

#define IR_LEFT 35
#define IR_RIGHT 34

#define MOTOR_SPEED 150

bfs::Mpu6500 imu;
Servo myServo;

#define SERVO_PIN 13
#define SDA_PIN 23
#define SCL_PIN 22

float alpha = 0.1;
float filteredAngle = 0;
float angleHistory[7] = {0};
int lastServoPosition = 90;

float getMPUAngle() {
    if (imu.Read()) {
        float accelY = imu.accel_y_mps2();
        float accelZ = imu.accel_z_mps2();
        float rawAngle = atan2(accelY, accelZ) * 180 / PI;
        filteredAngle = (alpha * rawAngle) + ((1 - alpha) * filteredAngle);

        for (int i = 6; i > 0; i--) {
            angleHistory[i] = angleHistory[i - 1];
        }
        angleHistory[0] = filteredAngle;

        float sum = 0;
        for (int i = 0; i < 7; i++) {
            sum += angleHistory[i];
        }
        float movingAvg = sum / 7.0;

        float sorted[7];
        memcpy(sorted, angleHistory, sizeof(angleHistory));
        for (int i = 0; i < 6; i++) {
            for (int j = i + 1; j < 7; j++) {
                if (sorted[i] > sorted[j]) {
                    float temp = sorted[i];
                    sorted[i] = sorted[j];
                    sorted[j] = temp;
                }
            }
        }
        float median = sorted[3];
        return (0.7 * median + 0.3 * movingAvg);
    }
    return 0;
}

void moveServoToZero() {
    int servoPosition = lastServoPosition;
    myServo.write(servoPosition);

    while (true) {
        float mpuAngle = getMPUAngle();
        if (abs(mpuAngle) < 0.3) break;

        float correction = mpuAngle * 0.3;
        int step = constrain(abs(correction), 1, 4);

        if (abs(mpuAngle) < 1.0 && abs(mpuAngle - getMPUAngle()) < 0.2) break;

        if (mpuAngle > 0) {
            servoPosition -= step;
        } else {
            servoPosition += step;
        }

        if (abs(servoPosition - lastServoPosition) > 3) {
            servoPosition = lastServoPosition + (servoPosition > lastServoPosition ? 2 : -2);
        }

        servoPosition = constrain(servoPosition, 0, 180);
        myServo.write(servoPosition);
        lastServoPosition = servoPosition;
        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

void TaskLineFollowing(void *pvParameters) {
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);

    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);

    while (1) {
        int leftSensor = digitalRead(IR_LEFT);
        int rightSensor = digitalRead(IR_RIGHT);

        if (leftSensor == 0 && rightSensor == 0) {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN4, HIGH);
            digitalWrite(IN3, LOW);
        } else if (leftSensor == 0 && rightSensor == 1) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        } else if (leftSensor == 1 && rightSensor == 0) {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
        }

        vTaskDelay(1);
    }
}

void TaskMPU(void *pvParameters) {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);

    if (!imu.Begin()) {
        while (1);
    }

    myServo.attach(SERVO_PIN);
    myServo.write(90);
    delay(500);

    while (1) {
        float currentAngle = getMPUAngle();
        if (abs(currentAngle) > 0.3) moveServoToZero();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);

    xTaskCreatePinnedToCore(
        TaskLineFollowing,
        "TaskLineFollowing",
        10000,
        NULL,
        1,
        &Task1,
        1
    );

    xTaskCreatePinnedToCore(
        TaskMPU,
        "TaskMPU",
        10000,
        NULL,
        1,
        &Task2,
        0
    );
}

void loop() {}
