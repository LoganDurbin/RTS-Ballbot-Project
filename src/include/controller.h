#pragma once
#include "config.h"

extern float integral_pitch, integral_roll;

struct WheelSpeeds
{
  int a, b, c;
};

inline WheelSpeeds compute(float pitch, float roll,
                    float pitch_rate, float roll_rate, float dt)
{

  // Update integrals
  integral_pitch += pitch * dt;
  integral_roll += roll * dt;

  // Clamp integrals to prevent windup
  integral_pitch = constrain(integral_pitch, -10.0f, 10.0f);
  integral_roll = constrain(integral_roll, -10.0f, 10.0f);

  float vx = KP * pitch + KI * integral_pitch + KD * pitch_rate;
  float vy = KP * roll + KI * integral_roll + KD * roll_rate;

  WheelSpeeds w;
  // A is aligned with the robot X axis; B/C are at ±120° around the X axis.
  w.a = (int)(vx);
  w.b = (int)(-0.5f * vx - 0.866f * vy);
  w.c = (int)(-0.5f * vx + 0.866f * vy);

  // clamp
  w.a = constrain(w.a, -MAX_SPEED, MAX_SPEED);
  w.b = constrain(w.b, -MAX_SPEED, MAX_SPEED);
  w.c = constrain(w.c, -MAX_SPEED, MAX_SPEED);

  return w;
}