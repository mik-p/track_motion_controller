#pragma once

#include <Arduino.h>

#include "boards/pin_defs.h"

#include "utils/version.h"

#include "platform/pid_defs.h"
#include "platform/encoder_defs.h"
#include "platform/motor_defs.h"
#include "platform/controller_defs.h"
#include "platform/interface_defs.h"

#include "console.h"

namespace tmc
{

#ifndef TMC_USE_CONSOLE
#define TMC_USE_CONSOLE 1
#endif

#ifndef TMC_DEBUG_LOGGING
#define TMC_DEBUG_LOGGING 0
#endif

bool tmc_use_logging = false;

/**
 * @brief console command functions
 *
 */

// console set debug string on/off command
int toggle_use_debug(int /*argc*/ = 0, char** /*argv*/ = NULL)
{
  tmc_use_logging = !tmc_use_logging;
  return EXIT_SUCCESS;
}

// console print version command
int console_print_version(int /*argc*/ = 0, char** /*argv*/ = NULL)
{
  print_version(&TMC_SERIAL_CONFIG, __FILE__);
  return EXIT_SUCCESS;
}

// set loop rate hz
int set_loop_hz(int argc, char** argv)
{
  if (argc != 2)
  {
    shell.println("bad argument count");
    return -1;
  }

  unsigned long loop_hz = atoi(argv[1]);

  if (loop_hz < 1 || loop_hz > 500)
  {
    shell.println("rate not in range");
    return -1;
  }

  shell.print("Setting loop frequency to ");
  shell.print(loop_hz);
  shell.println("hz");

  dt = ((double)(1.0 / loop_hz) * 1000);  // ms

  smc.set_update_interval(dt);

  return EXIT_SUCCESS;
}

// commands test encoders
// get encoder counts
int run_enc_test_loop(int argc = 0, char** argv = NULL)
{
  if (argc != 2)
  {
    shell.println("bad argument count");
    return -1;
  }

  // which encoder
  unsigned long enc = atoi(argv[1]);

  if (enc < 1 || enc > 4)
  {
    shell.println("encoder not in range");
    return -1;
  }

  shell.print("Testing encoder ");
  shell.print(enc);
  shell.println("...");

  // read out encoder pulses
  char inchar = shell.read();
  while(inchar != 'q')
  {
    shell.print("ENC");
    shell.print(enc);
    shell.print(":");
    shell.print(enc_ptr_arr[enc - 1]->get_log_string());
    shell.print(debug_enc_isr_trig_arr[enc - 1]);
    shell.println("\t'q' to quit");

    debug_enc_isr_trig_arr[enc - 1] = false;

    inchar = shell.read();

    delay(100);
  }

  shell.println();

  return EXIT_SUCCESS;
}

// commands test motors
// get top speed
int mx_test_effort(int argc = 0, char** argv = NULL)
{
  if (argc != 2)
  {
    shell.println("bad argument count");
    return -1;
  }

  // which motor
  unsigned long motor = atoi(argv[1]);

  if (motor < 1 || motor > 2)
  {
    shell.println("motor not in range");
    return -1;
  }

  shell.print("Testing motor ");
  shell.print(motor);
  shell.println("...");

  // test max speed for 2 seconds
  double speed = emc_array[motor - 1].test_effort_response(MOTOR_EFFORT_MAX, 2000);

  // print speed
  shell.print("speed: ");
  shell.println(speed);

  return EXIT_SUCCESS;
}

// get speed to effort scalar
int mx_tune(int argc = 0, char** argv = NULL)
{
  if (argc != 2)
  {
    shell.println("bad argument count");
    return -1;
  }

  // which motor
  unsigned long motor = atoi(argv[1]);

  if (motor < 1 || motor > 2)
  {
    shell.println("motor not in range");
    return -1;
  }

  shell.print("Tuning motor ");
  shell.print(motor);
  shell.println("...");

  // test max speed for 2 seconds
  double veleff = emc_array[motor - 1].tune_effort_scalar(MOTOR_EFFORT_MAX);

  // print speed
  shell.print("ratio: ");
  shell.println(veleff);

  return EXIT_SUCCESS;
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
  console.register_config(F("rate <loop_rate>"), set_loop_hz);
  // set pid variables

  // run special control commands
  // encoder tests
  console.register_config(F("encloop <enc_id>"), run_enc_test_loop);
  // motor controls
  console.register_config(F("mxeff <motor_id>"), mx_test_effort);
  console.register_config(F("mxtune <motor_id>"), mx_tune);

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
    TMC_SERIAL_CONFIG.println("TMC:DEBUG:" + smc.get_log_string());
  }
#endif
}

}  // namespace tmc
