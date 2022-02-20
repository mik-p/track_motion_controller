#pragma once

#include <Arduino.h>

#include "utils/version.h"
#include "utils/blink.h"

#include "platform/controller_defs.h"

namespace tmc
{

#ifndef TMC_DEBUG_LOGGING
#define TMC_DEBUG_LOGGING 0
#endif

#define TMC_STARTUP_DELAY 1500

/**
 * @brief arduinos setup function
 * opportunity to get the hardware initialized
 *
 */
void tmc_setup(const char* fname)
{
#if defined(TMC_E407)
  // begin config port
  TMC_SERIAL_CONFIG.begin(115200);
//   TMC_SERIAL_CONTROL.begin(115200);
#endif
  // say hi
  blink(100);
  blink(100);

  // init
  tmc_init_controller();

  // start up delay
  uint16_t del = TMC_STARTUP_DELAY;

  // print version
  blink(del);

  TMC_SERIAL_CONFIG.println("---   ###   ---");
  print_version(&TMC_SERIAL_CONFIG, fname);

  blink(del);

  print_version(&TMC_SERIAL_CONFIG, fname);
  TMC_SERIAL_CONFIG.println("---   ###   ---");
}

/**
 * @brief main control loop
 * LED blinks each time the loop passes for the duration of
 * the remaining time if the blink does not occur
 * it means that the loop over-ran
 *
 *
 */
void tmc_loop()
{
  // run motion control at full speed
  unsigned long start_time = millis();

  // controller loop
  smc.loop();

  // logging
#if TMC_DEBUG_LOGGING
  TMC_SERIAL_CONFIG.println(str);
#endif

  // delay the remaining time
  blink(get_loop_time_remaining(start_time) / 2);
}

}  // namespace tmc
