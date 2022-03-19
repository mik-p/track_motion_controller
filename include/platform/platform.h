#pragma once

#include <Arduino.h>

#include "utils/blink.h"

#include "platform/controller_defs.h"
#include "platform/console_defs.h"

namespace tmc
{

// start up delay
#define TMC_STARTUP_DELAY 1000

/**
 * @brief arduinos setup function
 * opportunity to get the hardware initialized
 *
 */
void tmc_setup(const char* fname)
{
  // say hi
  blink(100);
  blink(100);

  // init console
  setup_console_commands();

  // init
  tmc_init_controller();

  // print version
  blink(TMC_STARTUP_DELAY);
  log_show_version(fname);

  blink(TMC_STARTUP_DELAY);
  log_show_version(fname);
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
  loop_log();

  // check config shell
  loop_console();

  // delay the remaining time
  blink(get_loop_time_remaining(start_time) / 2);
}

}  // namespace tmc
