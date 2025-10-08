Project, junior year TSA, my contribution to team
Overview

This project is a custom-built drone system that integrates onboard flight stabilization, web-based remote control, and real-time video streaming — all using ESP32 microcontrollers.
It combines avionics, control systems, and embedded networking to demonstrate a miniature fly-by-wire and fly-by-web platform.

The drone is controlled entirely from a web interface hosted by the ESP32-CAM, which streams live video and sends movement or actuation commands to the ESP32-D via serial communication.
The ESP32-D manages flight dynamics, motor control, solenoid actuation, and stabilization using an MPU6050 IMU.

Component	Function
ESP32-CAM	Hosts the web server, streams live video, and sends control commands
ESP32-D	Controls motors, stabilization, and solenoid
MPU6050	Provides gyroscope and accelerometer data for flight stabilization
4 Motors (A, B, C, D)	Provide lift and control (brushless via ESCs)
4 ESCs (SoloGood)	Regulate power to each motor from the ESP32-D PWM signals
5V Electromagnetic Solenoid	Handles lift/release mechanism
L298N or XY-160D Motor Drivers	Interface motor PWM signals (if brushed)
Li-Po Battery (2S or 3S)	Power source for the entire system

Motor Config A C
            B D
Software Architecture
ESP32-CAM
Runs a web server with a live MJPEG video stream.
Generates HTML controls for flight operations.
Transmits control commands over Serial to the ESP32-D.

ESP32-D
Receives control data via Serial.
Reads MPU6050 data for real-time attitude feedback.
Adjusts motor PWM signals for pitch, roll, and yaw correction.

Activates/deactivates solenoid using PWM pin D5.
