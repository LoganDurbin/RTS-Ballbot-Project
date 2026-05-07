#pragma once

// --- I2C ---
#define IMU_SDA_PIN 4
#define IMU_SCL_PIN 5
#define IMU_ADDRESS 0x6A

// --- Motor Pins (IN1, IN2 per motor) ---
#define MOTOR_A_IN1 0
#define MOTOR_A_IN2 1
#define MOTOR_B_IN1 2
#define MOTOR_B_IN2 3
#define MOTOR_C_IN1 8
#define MOTOR_C_IN2 9

// --- Control Loop ---
#define LOOP_HZ 200
#define LOOP_US (1000000 / LOOP_HZ) // 5000 µs

// --- PD Gains (tune these) ---
#define KP 8.0f
#define KD 5.0f
#define KI 0.0f

// --- Motor Output ---
#define MAX_SPEED 80
#define DEADBAND 4 // below this PWM, motor doesn't move — set after testing

// --- Demo / Debug ---
#define INJECT_DELAY_MS 0 // set to e.g. 20 to demonstrate instability
#define SERIAL_BAUD 115200