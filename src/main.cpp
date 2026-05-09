#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "motors.h"
#include "controller.h"
#include "timing.h"
#include "data_logger.h"

// imu.h globals
float gyro_offset_x = 0, gyro_offset_y = 0;
float pitch = 0, roll = 0;
unsigned long last_time_us = 0;

// timing.h globals
unsigned long loop_start_us = 0;
unsigned long last_elapsed_us = 0;
unsigned long max_elapsed_us = 0;

// data logging
DataLogger logger;
unsigned long log_start_ms = 0;
unsigned long log_entry_count = 0;

// PID integral accumulators
float pitch_error_integral = 0.0f;
float roll_error_integral = 0.0f;
#define MAX_INTEGRAL 50.0f  // anti-windup clamp

Motor motorA, motorB, motorC;
LSM6DS3 imu;

void stop_all_motors()
{
  motor_set(motorA, 0);
  motor_set(motorB, 0);
  motor_set(motorC, 0);
}

bool is_upside_down(const IMUData &d)
{
  // The IMU is mounted inverted, and read() corrects the Z sign.
  // For a right-side-up robot, corrected Z is near +1g.
  // When the robot flips upside down, corrected Z falls near -1g.
  return d.az < -0.8f;
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Wire.begin();
  Wire.setClock(400000);

  imu.begin();
  delay(200);
  imu.calibrate(200);
  last_time_us = micros();

  motor_init(motorA, MOTOR_A_IN1, MOTOR_A_IN2);
  motor_init(motorB, MOTOR_B_IN1, MOTOR_B_IN2);
  motor_init(motorC, MOTOR_C_IN1, MOTOR_C_IN2);

  stop_all_motors();

  // Initialize data logger
  if (logger.begin()) {
    log_start_ms = millis();
    Serial.println("Data logging enabled.");
  } else {
    Serial.println("WARNING: Data logging failed to initialize.");
  }

  Serial.println("Ready.");
}

void loop()
{
  static unsigned long next_loop_us = micros();
  static int last_motor_a = 0, last_motor_b = 0, last_motor_c = 0;

  timing_start();

  IMUData d = imu.read();

  float p, r;
  imu.estimate_tilt(d, p, r);

  // Accumulate integral error with anti-windup
  unsigned long now_us = micros();
  static unsigned long last_integral_update_us = now_us;
  float dt_integral = (now_us - last_integral_update_us) / 1e6f;
  if (dt_integral > 0.05f) dt_integral = 0.05f;  // clamp dt
  last_integral_update_us = now_us;

  pitch_error_integral += p * dt_integral;
  roll_error_integral += r * dt_integral;

  // Clamp integral to prevent windup
  pitch_error_integral = constrain(pitch_error_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  roll_error_integral = constrain(roll_error_integral, -MAX_INTEGRAL, MAX_INTEGRAL);

  int motor_a_cmd = 0, motor_b_cmd = 0, motor_c_cmd = 0;

  if (is_upside_down(d))
  {
    stop_all_motors();
    // Reset integral accumulators when robot is down
    pitch_error_integral = 0.0f;
    roll_error_integral = 0.0f;
  }
  else
  {
    WheelSpeeds w = compute(p, r, d.gx, d.gy, pitch_error_integral, roll_error_integral);

    motor_a_cmd = w.a;
    motor_b_cmd = w.b;
    motor_c_cmd = w.c;

    motor_set(motorA, w.a);
    motor_set(motorB, w.b);
    motor_set(motorC, w.c);

    last_motor_a = w.a;
    last_motor_b = w.b;
    last_motor_c = w.c;
  }

  timing_end();

  // Log data to flash
  if (logger.is_ready()) {
    unsigned long current_ms = millis() - log_start_ms;
    float tilt_error = sqrtf(p * p + r * r); // magnitude of tilt angle
    bool timing_miss = (last_elapsed_us > LOOP_US);

    logger.log_entry(
        current_ms,
        last_motor_a, last_motor_b, last_motor_c,
        p, r, tilt_error,
        d,
        last_elapsed_us,
        max_elapsed_us,
        timing_miss);

    log_entry_count++;
  }

  timing_print_stats(); // comment out once tuning — adds latency

  timing_wait(next_loop_us);
}

// Optional: Call this to stop logging and flush data to flash
// Can be triggered by serial command, button press, or time limit
void stop_logging()
{
  if (logger.is_ready()) {
    logger.end();
    Serial.println("Logging stopped.");
  }
}

// Optional: Clear all logged data and start fresh
void clear_all_logs()
{
  DataLogger::clear_logs();
  Serial.println("All logs cleared.");
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