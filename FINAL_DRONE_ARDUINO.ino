/*
#include <Wire.h>
#include <MPU6050.h>

#define MOTOR_A 3
#define MOTOR_B 10
#define MOTOR_C 6
#define MOTOR_D 9
#define SOLENOID 5

MPU6050 mpu;
float pitch, roll;
int motorSpeed = 150;  // Adjust for stability

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();

    pinMode(MOTOR_A, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);
    pinMode(MOTOR_C, OUTPUT);
    pinMode(MOTOR_D, OUTPUT);
    pinMode(SOLENOID, OUTPUT);

    Serial.println("Nano Ready!");
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    pitch = atan2(ay, az) * 180 / PI;
    roll = atan2(ax, az) * 180 / PI;

    int adjA = motorSpeed - roll + pitch;
    int adjB = motorSpeed + roll - pitch;
    int adjC = motorSpeed - roll - pitch;
    int adjD = motorSpeed + roll + pitch;

    adjA = constrain(adjA, 0, 255);
    adjB = constrain(adjB, 0, 255);
    adjC = constrain(adjC, 0, 255);
    adjD = constrain(adjD, 0, 255);

    analogWrite(MOTOR_A, adjA);
    analogWrite(MOTOR_B, adjB);
    analogWrite(MOTOR_C, adjC);
    analogWrite(MOTOR_D, adjD);

    if (Serial.available()) {
        char command = Serial.read();
        if (command == 'S') digitalWrite(SOLENOID, HIGH);  // Activate solenoid
        if (command == 'X') digitalWrite(SOLENOID, LOW);   // Deactivate solenoid
    }

    delay(10);
}
*/
/*
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define MOTOR_A 3
#define MOTOR_B 10
#define MOTOR_C 6
#define MOTOR_D 9
#define SOLENOID 5

Adafruit_MPU6050 mpu;
float pitch, roll;
int motorSpeed = 150;  // Adjust for stability

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Try to initialize!
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    
    // Setup the accelerometer range
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    // Setup the gyroscope range
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    // Setup filter bandwidth
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    pinMode(MOTOR_A, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);
    pinMode(MOTOR_C, OUTPUT);
    pinMode(MOTOR_D, OUTPUT);
    pinMode(SOLENOID, OUTPUT);

    Serial.println("Nano Ready!");
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate pitch and roll from accelerometer data
    pitch = atan2(a.acceleration.y, sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / PI;
    roll = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI;

    int adjA = motorSpeed - roll + pitch;
    int adjB = motorSpeed + roll - pitch;
    int adjC = motorSpeed - roll - pitch;
    int adjD = motorSpeed + roll + pitch;

    adjA = constrain(adjA, 0, 255);
    adjB = constrain(adjB, 0, 255);
    adjC = constrain(adjC, 0, 255);
    adjD = constrain(adjD, 0, 255);

    analogWrite(MOTOR_A, adjA);
    analogWrite(MOTOR_B, adjB);
    analogWrite(MOTOR_C, adjC);
    analogWrite(MOTOR_D, adjD);

    if (Serial.available()) {
        char command = Serial.read();
        if (command == 'S') digitalWrite(SOLENOID, HIGH);  // Activate solenoid
        if (command == 'X') digitalWrite(SOLENOID, LOW);   // Deactivate solenoid
    }

    delay(10);
}
*/
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define MOTOR_A 3
#define MOTOR_B 10
#define MOTOR_C 6
#define MOTOR_D 9
#define SOLENOID 5

Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(115200); // Communication with ESP32-CAM
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX2=16, TX2=17

    pinMode(MOTOR_A, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);
    pinMode(MOTOR_C, OUTPUT);
    pinMode(MOTOR_D, OUTPUT);
    pinMode(SOLENOID, OUTPUT);

    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1);
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float pitch = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
    float roll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;

    int speedA = constrain(1500 + (pitch * 10), 1000, 2000);
    int speedB = constrain(1500 - (pitch * 10), 1000, 2000);
    int speedC = constrain(1500 + (roll * 10), 1000, 2000);
    int speedD = constrain(1500 - (roll * 10), 1000, 2000);

    analogWrite(MOTOR_A, speedA);
    analogWrite(MOTOR_B, speedB);
    analogWrite(MOTOR_C, speedC);
    analogWrite(MOTOR_D, speedD);

    if (Serial2.available()) {
        String command = Serial2.readStringUntil('\n');
        command.trim();

        if (command == "UP") {
            analogWrite(MOTOR_A, 2000);
            analogWrite(MOTOR_B, 2000);
            analogWrite(MOTOR_C, 2000);
            analogWrite(MOTOR_D, 2000);
        } else if (command == "DOWN") {
            analogWrite(MOTOR_A, 1000);
            analogWrite(MOTOR_B, 1000);
            analogWrite(MOTOR_C, 1000);
            analogWrite(MOTOR_D, 1000);
        } else if (command == "SPIN_RIGHT") {
            analogWrite(MOTOR_A, 1800);
            analogWrite(MOTOR_B, 1200);
            analogWrite(MOTOR_C, 1200);
            analogWrite(MOTOR_D, 1800);
        } else if (command == "SPIN_LEFT") {
            analogWrite(MOTOR_A, 1200);
            analogWrite(MOTOR_B, 1800);
            analogWrite(MOTOR_C, 1800);
            analogWrite(MOTOR_D, 1200);
        } else if (command == "CHARGE_SOLENOID") {
            digitalWrite(SOLENOID, HIGH);
        } else if (command == "DISCHARGE_SOLENOID") {
            digitalWrite(SOLENOID, LOW);
        }
    }

    delay(10);
}
