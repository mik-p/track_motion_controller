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

#if defined(TMC_E407)
  digitalWrite(LED_BUILTIN, LOW);
#else
  digitalWrite(LED_BUILTIN, HIGH);
#endif
  delay(ms);
#if defined(TMC_E407)
  digitalWrite(LED_BUILTIN, HIGH);
#else
  digitalWrite(LED_BUILTIN, LOW);
#endif
  delay(ms);
}
