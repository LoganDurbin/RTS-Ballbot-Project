#pragma once
#include "config.h"

struct WheelSpeeds
{
  int a, b, c;
};

inline WheelSpeeds compute(float pitch, float roll,
                    float pitch_rate, float roll_rate)
{
  float vx = -(KP * pitch + KD * pitch_rate);
  float vy = -(KP * roll + KD * roll_rate);

  WheelSpeeds w;
  w.a = (int)(vy);
  w.b = (int)(-0.866f * vx - 0.5f * vy);
  w.c = (int)(0.866f * vx - 0.5f * vy);

  // clamp
  w.a = constrain(w.a, -MAX_SPEED, MAX_SPEED);
  w.b = constrain(w.b, -MAX_SPEED, MAX_SPEED);
  w.c = constrain(w.c, -MAX_SPEED, MAX_SPEED);

  return w;
}