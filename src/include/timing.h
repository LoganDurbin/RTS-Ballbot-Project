#pragma once
#include <Arduino.h>
#include "config.h"

extern unsigned long loop_start_us;
extern unsigned long last_elapsed_us;
extern unsigned long max_elapsed_us;

inline void timing_start()
{
  loop_start_us = micros();
}

inline void timing_end()
{
  last_elapsed_us = micros() - loop_start_us;
  if (last_elapsed_us > max_elapsed_us)
    max_elapsed_us = last_elapsed_us;
}

inline void timing_wait(unsigned long &next_loop_us)
{
  if (INJECT_DELAY_MS > 0)
    delay(INJECT_DELAY_MS);

  while (micros() < next_loop_us)
    ;
  next_loop_us += LOOP_US;
}

inline void timing_print_stats()
{
  Serial.print("loop_us: ");
  Serial.print(last_elapsed_us);
  Serial.print("  max_us: ");
  Serial.println(max_elapsed_us);
}