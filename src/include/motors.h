#pragma once
#include <Arduino.h>
#include "config.h"

struct Motor
{
  uint8_t in1, in2;
};

inline void motor_init(Motor &m, uint8_t pin1, uint8_t pin2)
{
  m.in1 = pin1;
  m.in2 = pin2;
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
}

inline void motor_set(Motor &m, int speed)
{
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (abs(speed) < DEADBAND)
    speed = 0;
  if (speed > 0)
  {
    analogWrite(m.in1, speed);
    analogWrite(m.in2, 0);
  }
  else if (speed < 0)
  {
    analogWrite(m.in1, 0);
    analogWrite(m.in2, -speed);
  }
  else
  {
    analogWrite(m.in1, 0);
    analogWrite(m.in2, 0);
  }
}