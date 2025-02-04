#define motorA 3  // PWM pin for Motor A
#define motorB 10  // PWM pin for Motor B
#define motorC 6  // PWM pin for Motor C
#define motorD 9  // PWM pin for Motor D

void setup() {
    pinMode(motorA, OUTPUT);
    pinMode(motorB, OUTPUT);
    pinMode(motorC, OUTPUT);
    pinMode(motorD, OUTPUT);
}

void loop() {
    int hoverSpeed = 255;  // Adjust PWM value for stable hover

    analogWrite(motorA, hoverSpeed);  // Motor A (CCW)
    analogWrite(motorB, hoverSpeed);  // Motor B (CCW)
    analogWrite(motorC, hoverSpeed);  // Motor C (CW)
    analogWrite(motorD, hoverSpeed);  // Motor D (CW)
}

