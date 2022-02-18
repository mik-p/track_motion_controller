#pragma once

#include <Arduino.h>

#include "boards/pin_defs.h"

/**
 * @brief blink an led for a given ms duration
 *
 */
void blink(uint32_t ms)
{
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(ms);
  digitalWrite(LED_BUILTIN, LOW);
  delay(ms);
}
