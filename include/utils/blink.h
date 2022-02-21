#pragma once

#include <Arduino.h>

#include "boards/pin_defs.h"

#define BLINK_MAX_LEN_MS 2000

/**
 * @brief blink an led for a given ms duration
 *
 */
void blink(unsigned long ms)
{
  // guard maximum duration
  if (ms > BLINK_MAX_LEN_MS)
  {
    ms = BLINK_MAX_LEN_MS;
  }

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
