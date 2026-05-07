#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "motors.h"
#include "controller.h"
#include "timing.h"

// imu.h globals
float pitch = 0, roll = 0;
unsigned long last_time_us = 0;
LSM6DS3 imu;

// controller globals
float integral_pitch = 0, integral_roll = 0;
unsigned long last_loop_us = 0;

// timing.h globals
unsigned long loop_start_us = 0;
unsigned long last_elapsed_us = 0;
unsigned long max_elapsed_us = 0;


Motor motorA, motorB, motorC;

void setup()
{
  Serial.begin(115200);
  delay(1000); // wait for Serial to initialize
  Serial.println("Serial good.");
  Wire.begin();
  Wire.setClock(400000);
  delay(5000);
  if (!imu.begin()) {
    Serial.println("IMU init failed");
    while (true) {
      delay(1000);
    }
  }
  last_time_us = micros();

  motor_init(motorA, MOTOR_A_IN1, MOTOR_A_IN2);
  motor_init(motorB, MOTOR_B_IN1, MOTOR_B_IN2);
  motor_init(motorC, MOTOR_C_IN1, MOTOR_C_IN2);

  Serial.println("Ready.");
}

void motor_test_sequence()
 {
   Serial.println("Motor A forward");
   motor_set(motorA, 150);
   delay(1000);
   motor_set(motorA, 0);
   delay(500);

   Serial.println("Motor A reverse");
   motor_set(motorA, -150);
   delay(1000);
   motor_set(motorA, 0);
   delay(500);
 
   Serial.println("Motor B forward");
   motor_set(motorB, 150);
   delay(1000);
   motor_set(motorB, 0);
   delay(500);

   Serial.println("Motor B reverse");
   motor_set(motorB, -150);
   delay(1000);
   motor_set(motorB, 0);
   delay(500);

   Serial.println("Motor C forward");
   motor_set(motorC, 150);
   delay(1000);
   motor_set(motorC, 0);
   delay(500);


  Serial.println("Motor C reverse");
   motor_set(motorC, -150);
   delay(1000);
   motor_set(motorC, 0);
   delay(500);
 }

void loop()
{
  static unsigned long next_loop_us = micros();

  unsigned long current_us = micros();
  float dt = (current_us - last_loop_us) / 1e6f;
  if (dt > 0.01f) dt = 0.01f;
  last_loop_us = current_us;

  timing_start();

  IMUData d = imu.read();
  float p, r;
  imu.estimate_tilt(d, p, r);

  WheelSpeeds w = compute(p, r, d.gx, d.gy, dt);

  motor_set(motorA, w.a);
  motor_set(motorB, w.b);
  motor_set(motorC, w.c);

  timing_end();
  //timing_print_stats(); // comment out once tuning — adds latency

  timing_wait(next_loop_us);
  
  //motor_test_sequence();
} 