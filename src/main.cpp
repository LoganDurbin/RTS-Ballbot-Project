#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "motors.h"
#include "controller.h"
#include "timing.h"

// imu.h globals
float gyro_offset_x = 0, gyro_offset_y = 0;
float pitch = 0, roll = 0;
unsigned long last_time_us = 0;

// timing.h globals
unsigned long loop_start_us = 0;
unsigned long last_elapsed_us = 0;
unsigned long max_elapsed_us = 0;

arduino::MbedI2C imu_wire(IMU_SDA_PIN, IMU_SCL_PIN);

Motor motorA, motorB, motorC;

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  imu_wire.begin();
  imu_wire.setClock(400000);

  lsm_init();
  delay(200);
  calibrate_gyro(200);
  last_time_us = micros();

  motor_init(motorA, MOTOR_A_IN1, MOTOR_A_IN2);
  motor_init(motorB, MOTOR_B_IN1, MOTOR_B_IN2);
  motor_init(motorC, MOTOR_C_IN1, MOTOR_C_IN2);

  Serial.println("Ready.");
}

void loop()
{
  static unsigned long next_loop_us = micros();

  timing_start();

  IMUData d = lsm_get();
  float p, r;
  estimate_tilt(d, p, r);

  WheelSpeeds w = compute(p, r, d.gx, d.gy);

  motor_set(motorA, w.a);
  motor_set(motorB, w.b);
  motor_set(motorC, w.c);

  timing_end();
  timing_print_stats(); // comment out once tuning — adds latency

  timing_wait(next_loop_us);
}

// motor_test.cpp — standalone test, comment out main.cpp loop to use
// void motor_test_sequence()
// {
//   Serial.println("Motor A forward");
//   motor_set(motorA, 150);
//   delay(1000);
//   motor_set(motorA, 0);
//   delay(500);

//   Serial.println("Motor A reverse");
//   motor_set(motorA, -150);
//   delay(1000);
//   motor_set(motorA, 0);
//   delay(500);

//   Serial.println("Motor B forward");
//   motor_set(motorB, 150);
//   delay(1000);
//   motor_set(motorB, 0);
//   delay(500);

//   Serial.println("Motor B reverse");
//   motor_set(motorB, -150);
//   delay(1000);
//   motor_set(motorB, 0);
//   delay(500);

//   Serial.println("Motor C forward");
//   motor_set(motorC, 150);
//   delay(1000);
//   motor_set(motorC, 0);
//   delay(500);

//   Serial.println("Motor C reverse");
//   motor_set(motorC, -150);
//   delay(1000);
//   motor_set(motorC, 0);
//   delay(500);
// }