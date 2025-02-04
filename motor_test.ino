int motorA = 3;  // PWM pin for Motor A
int motorB = 10;  // PWM pin for Motor B
int motorC = 6;  // PWM pin for Motor C
int motorD = 9;  // PWM pin for Motor D

void setup() {
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(motorC, OUTPUT);
  pinMode(motorD, OUTPUT);
}

void loop() {
  analogWrite(motorA, 150); // Test Motor A
  analogWrite(motorB, 150); // Test Motor B
  analogWrite(motorC, 150); // Test Motor C
  analogWrite(motorD, 150); // Test Motor D
  delay(5000);              // Run for 5 seconds

  analogWrite(motorA, 0);
  analogWrite(motorB, 0);
  analogWrite(motorC, 0);
  analogWrite(motorD, 0);
  delay(5000);              // Stop for 5 seconds
}

