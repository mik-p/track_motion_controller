#pragma once

#include <Arduino.h>

#include "boards/pin_defs.h"

#include "utils/version.h"

#include "platform/controller_defs.h"

#include "console.h"

namespace tmc
{

#ifndef TMC_USE_CONSOLE
#define TMC_USE_CONSOLE 1
#endif

#ifndef TMC_DEBUG_LOGGING
#define TMC_DEBUG_LOGGING 0
#endif

bool tmc_use_logging = TMC_DEBUG_LOGGING;

// console set debug string on/off command
int toggle_use_debug(int /*argc*/ = 0, char** /*argv*/ = NULL)
{
  tmc_use_logging = !tmc_use_logging;
}

// console print version command
int console_print_version(int /*argc*/ = 0, char** /*argv*/ = NULL)
{
  print_version(&TMC_SERIAL_CONFIG, __FILE__);
  return 0;
}

#if TMC_USE_CONSOLE
// interactive console insance
Console console = Console(&TMC_SERIAL_CONFIG, 115200);
#endif

// init interactive console functions
void setup_console_commands()
{
#if TMC_USE_CONSOLE
  // init serial console
  console.init(&TMC_SERIAL_CONFIG);

  // add commands
  // version
  console.register_config(F("version"), console_print_version);
  // set debug string on/off
  console.register_config(F("debug"), toggle_use_debug);
  // set comms config
  // set loop rate and other 'constants'
  // set pid variables
#endif
}

// loop the interactive console
void loop_console()
{
#if TMC_USE_CONSOLE
  // check config shell
  console.loop();
#endif
}

// show version info
void log_show_version(const char* fname)
{
  TMC_SERIAL_CONFIG.println("---   ###   ---");
  print_version(&TMC_SERIAL_CONFIG, fname);
  TMC_SERIAL_CONFIG.println("---   ###   ---");
}

// loop logging
void loop_log()
{
#if TMC_DEBUG_LOGGING
  if (tmc_use_logging)
  {
    TMC_SERIAL_CONFIG.println(smc.get_log_string());
  }
#endif
}

}  // namespace tmc
