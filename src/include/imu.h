#pragma once
#include "config.h"
#include <Wire.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define LSM_ADDR 0x6A  // or 0x6B if SA0 pin is high
#define WHO_AM_I 0x0F  // should return 0x69
#define CTRL1_XL 0x10  // accel config
#define CTRL2_G 0x11   // gyro config
#define OUTX_L_G 0x22  // gyro data start
#define OUTX_L_XL 0x28 // accel data start

// Scale factors (matching config below)
// Accel: ±2g → 0.061 mg/LSB
// Gyro:  ±250°/s → 8.75 mdps/LSB
#define ACCEL_SCALE 0.000061f // g per LSB
#define GYRO_SCALE 0.00875f   // deg/s per LSB

extern float gyro_offset_x, gyro_offset_y;
extern float pitch, roll;
extern unsigned long last_time_us;
extern arduino::MbedI2C imu_wire;

inline void lsm_write(uint8_t reg, uint8_t val)
{
  imu_wire.beginTransmission(LSM_ADDR);
  imu_wire.write(reg);
  imu_wire.write(val);
  imu_wire.endTransmission();
}

inline uint8_t lsm_read_byte(uint8_t reg)
{
  imu_wire.beginTransmission(LSM_ADDR);
  imu_wire.write(reg);
  imu_wire.endTransmission(false);
  imu_wire.requestFrom((uint8_t)LSM_ADDR, (size_t)1);
  return imu_wire.read();
}

inline void lsm_read_bytes(uint8_t reg, uint8_t *buf, int len)
{
  imu_wire.beginTransmission(LSM_ADDR);
  imu_wire.write(reg);
  imu_wire.endTransmission(false);
  imu_wire.requestFrom((uint8_t)LSM_ADDR, (size_t)len);
  for (int i = 0; i < len; i++)
    buf[i] = imu_wire.read();
}

inline void lsm_init()
{
  // Accel: 416 Hz, ±2g
  lsm_write(CTRL1_XL, 0x60);
  // Gyro:  416 Hz, ±250°/s
  lsm_write(CTRL2_G, 0x60);
  delay(100);

  // Verify chip
  uint8_t id = lsm_read_byte(WHO_AM_I);
  if (id != 0x69)
  {
    Serial.print("LSM6DS3 not found! WHO_AM_I=0x");
    Serial.println(id, HEX);
    while (true)
      ; // halt — wiring issue
  }
  Serial.println("LSM6DS3 OK");
}

struct IMUData
{
  float ax, ay, az; // g
  float gx, gy;     // deg/s (bias-corrected)
};

inline IMUData lsm_get()
{
  uint8_t buf[6];
  IMUData d;

  lsm_read_bytes(OUTX_L_G, buf, 6);
  int16_t raw_gx = (buf[1] << 8) | buf[0];
  int16_t raw_gy = (buf[3] << 8) | buf[2];
  d.gx = raw_gx * GYRO_SCALE - gyro_offset_x;
  d.gy = raw_gy * GYRO_SCALE - gyro_offset_y;

  lsm_read_bytes(OUTX_L_XL, buf, 6);
  int16_t raw_ax = (buf[1] << 8) | buf[0];
  int16_t raw_ay = (buf[3] << 8) | buf[2];
  int16_t raw_az = (buf[5] << 8) | buf[4];
  d.ax = raw_ax * ACCEL_SCALE;
  d.ay = raw_ay * ACCEL_SCALE;
  d.az = raw_az * ACCEL_SCALE;

  return d;
}

inline void calibrate_gyro(int samples)
{
  Serial.println("Calibrating — keep still...");
  float sx = 0, sy = 0;
  for (int i = 0; i < samples; i++)
  {
    IMUData d = lsm_get();
    sx += d.gx;
    sy += d.gy;
    delay(5);
  }
  gyro_offset_x = sx / samples;
  gyro_offset_y = sy / samples;
  Serial.println("Calibration done.");
}

inline void estimate_tilt(IMUData &d, float &out_pitch, float &out_roll)
{
  unsigned long now = micros();
  float dt = (now - last_time_us) / 1e6f;
  last_time_us = now;

  // Clamp dt — handles first call or stall
  if (dt > 0.05f)
    dt = 0.05f;

  float accel_pitch = atan2f(d.ay, d.az) * 180.0f / M_PI;
  float accel_roll = atan2f(d.ax, d.az) * 180.0f / M_PI;

  // Complementary filter
  pitch = 0.98f * (pitch + d.gx * dt) + 0.02f * accel_pitch;
  roll = 0.98f * (roll + d.gy * dt) + 0.02f * accel_roll;

  out_pitch = pitch;
  out_roll = roll;
}