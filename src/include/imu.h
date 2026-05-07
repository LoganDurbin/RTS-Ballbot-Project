#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =====================
// LSM6DS3 REGISTERS
// =====================
#define LSM6DS3_ADDR_1 0x6A
#define LSM6DS3_ADDR_2 0x6B

#define WHO_AM_I_REG   0x0F
#define WHO_AM_I_VALUE  0x69

#define CTRL1_XL  0x10
#define CTRL2_G   0x11
#define CTRL3_C   0x12

#define OUTX_L_G  0x22
#define OUTX_L_XL 0x28

// =====================
// SCALE FACTORS
// =====================
#define ACCEL_SCALE_2G   0.000061f   // g/LSB
#define GYRO_SCALE_250DPS 0.00875f   // dps/LSB
#define IMU_ACCEL_FILTER_ALPHA 0.12f  // low-pass accel filter
#define IMU_COMPLEMENTARY_ALPHA 0.96f // more stable tilt fusion

// =====================
// STATE
// =====================
struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
};

class LSM6DS3 {
public:
  inline void estimate_tilt(IMUData &d, float &out_pitch, float &out_roll)
{
  unsigned long now = micros();
  float dt = (now - last_time_us) / 1e6f;
  last_time_us = now;

  // Clamp dt — handles first call or stall
  if (dt > 0.05f)
    dt = 0.05f;

  float accel_pitch = atan2f(d.ay, d.az) * 180.0f / M_PI - pitch_bias;
  float accel_roll = atan2f(d.ax, d.az) * 180.0f / M_PI - roll_bias;

  // Complementary filter with more stable accel weighting
  pitch = IMU_COMPLEMENTARY_ALPHA * (pitch + d.gx * dt) + (1.0f - IMU_COMPLEMENTARY_ALPHA) * accel_pitch;
  roll = IMU_COMPLEMENTARY_ALPHA * (roll + d.gy * dt) + (1.0f - IMU_COMPLEMENTARY_ALPHA) * accel_roll;

  out_pitch = pitch;
  out_roll = roll;
}

  void calibrate(int samples = 50) {
    float sum_pitch = 0.0f;
    float sum_roll = 0.0f;
    for (int i = 0; i < samples; i++) {
      IMUData d = read();
      float accel_pitch = atan2f(d.ay, d.az) * 180.0f / M_PI;
      float accel_roll = atan2f(d.ax, d.az) * 180.0f / M_PI;
      sum_pitch += accel_pitch;
      sum_roll += accel_roll;
      delay(20);
    }
    pitch_bias = sum_pitch / samples;
    roll_bias = sum_roll / samples;
  }

  bool begin(TwoWire &wire = Wire) {
    _wire = &wire;

    // ---- I2C init (IMPORTANT) ----
    _wire->begin();
    _wire->setClock(400000);
    pitch = 0.0f;
    roll = 0.0f;
    last_time_us = micros();

    delay(20);

    // ---- detect device ----
    if (!detect()) {
      Serial.println("LSM6DS3 not found");
      return false;
    }

    // ---- soft reset + BDU ----
    writeReg(CTRL3_C, 0x44);  // BDU + auto-increment
    delay(10);

    // ---- configure accel (416 Hz, ±2g) ----
    writeReg(CTRL1_XL, 0x60);

    // ---- configure gyro (416 Hz, 250 dps) ----
    writeReg(CTRL2_G, 0x60);

    delay(50);

    calibrate();

    Serial.println("LSM6DS3 initialized OK");
    return true;
  }

  IMUData read() {
    IMUData d;

    uint8_t buf[12];

    // ---- read gyro + accel in one burst ----
    readBytes(OUTX_L_G, buf, 12);

    int16_t gx = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t gy = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t gz = (int16_t)(buf[5] << 8 | buf[4]);

    int16_t ax = (int16_t)(buf[7] << 8 | buf[6]);
    int16_t ay = (int16_t)(buf[9] << 8 | buf[8]);
    int16_t az = (int16_t)(buf[11] << 8 | buf[10]);

    d.gx = gx * GYRO_SCALE_250DPS;
    d.gy = gy * GYRO_SCALE_250DPS;
    d.gz = -gz * GYRO_SCALE_250DPS;  // Inverted for upside-down mount

    float raw_ax = ax * ACCEL_SCALE_2G;
    float raw_ay = ay * ACCEL_SCALE_2G;
    float raw_az = -az * ACCEL_SCALE_2G; // Inverted for upside-down mount

    if (!accel_initialized) {
      filtered_ax = raw_ax;
      filtered_ay = raw_ay;
      filtered_az = raw_az;
      accel_initialized = true;
    } else {
      filtered_ax = IMU_ACCEL_FILTER_ALPHA * raw_ax + (1.0f - IMU_ACCEL_FILTER_ALPHA) * filtered_ax;
      filtered_ay = IMU_ACCEL_FILTER_ALPHA * raw_ay + (1.0f - IMU_ACCEL_FILTER_ALPHA) * filtered_ay;
      filtered_az = IMU_ACCEL_FILTER_ALPHA * raw_az + (1.0f - IMU_ACCEL_FILTER_ALPHA) * filtered_az;
    }

    d.ax = filtered_ax;
    d.ay = filtered_ay;
    d.az = filtered_az;

    return d;
  }

private:
  TwoWire *_wire;
  uint8_t _addr;
  float pitch = 0.0f;
  float roll = 0.0f;
  float pitch_bias = 0.0f;
  float roll_bias = 0.0f;
  float filtered_ax = 0.0f;
  float filtered_ay = 0.0f;
  float filtered_az = 0.0f;
  bool accel_initialized = false;
  unsigned long last_time_us = 0;

  bool detect() {
    // try both addresses
    if (ping(LSM6DS3_ADDR_1)) {
      _addr = LSM6DS3_ADDR_1;
      return true;
    }
    if (ping(LSM6DS3_ADDR_2)) {
      _addr = LSM6DS3_ADDR_2;
      return true;
    }
    return false;
  }

  bool ping(uint8_t addr) {
    _wire->beginTransmission(addr);
    _wire->write(WHO_AM_I_REG);
    if (_wire->endTransmission(false) != 0) return false;

    if (_wire->requestFrom(addr, (uint8_t)1) != 1) return false;

    return (_wire->read() == WHO_AM_I_VALUE);
  }

  void writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
  }

  void readBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);

    if (_wire->endTransmission(false) != 0) {
      memset(buf, 0, len);
      return;
    }

    if (_wire->requestFrom(_addr, len) != len) {
      memset(buf, 0, len);
      return;
    }

    for (uint8_t i = 0; i < len; i++) {
      buf[i] = _wire->read();
    }
  }
};