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

void print_bad_arg_count()
{
  shell.println("bad argument count");
}

void print_index_not_in_range()
{
  shell.println("index not in range");
}

void print_append_q_quit()
{
  shell.println("\t'q' to quit");
}

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

// print battery voltage
int console_print_battery(int /*argc*/ = 0, char** /*argv*/ = NULL)
{
  shell.print("vbat: ");
  shell.println(battery.read());
  return EXIT_SUCCESS;
}

// set loop rate hz
int set_loop_hz(int argc, char** argv)
{
  if (argc != 2)
  {
    print_bad_arg_count();
    return -1;
  }

  unsigned long loop_hz = atoi(argv[1]);

  if (loop_hz < 1 || loop_hz > 500)
  {
    print_index_not_in_range();
    return -1;
  }

  shell.print("set loop hz ");
  shell.print(loop_hz);
  shell.println("hz");

  dt = ((double)(1.0 / loop_hz) * 1000);  // ms

  smc.set_update_interval(dt);

  return EXIT_SUCCESS;
}

// command test e-stop
int set_estop(int argc = 0, char** argv = NULL)
{
  if (argc != 2)
  {
    print_bad_arg_count();
    return -1;
  }

  // which setting
  unsigned long estop = atoi(argv[1]);

  if (estop < 0 || estop > 1)
  {
    shell.println("0 or 1");
    return -1;
  }

  shell.print("set e-stop ");
  shell.println(estop);
  digitalWrite(E_STOP_PIN, estop);

  return EXIT_SUCCESS;
}

// commands test encoders
// get encoder counts
int run_enc_test_loop(int argc = 0, char** argv = NULL)
{
  if (argc != 2)
  {
    print_bad_arg_count();
    return -1;
  }

  // which encoder
  unsigned long enc = atoi(argv[1]);

  if (enc < 1 || enc > 4)
  {
    print_index_not_in_range();
    return -1;
  }

  shell.print("test encoder ");
  shell.println(enc);

  // read out encoder pulses
  char inchar = shell.read();
  while(inchar != 'q')
  {
    shell.print("ENC");
    shell.print(enc);
    shell.print(":");
    shell.print(enc_ptr_arr[enc - 1]->get_log_string());
    shell.print(debug_enc_isr_trig_arr[enc - 1]);
    print_append_q_quit();

    debug_enc_isr_trig_arr[enc - 1] = false;

    inchar = shell.read();

    delay(100);
  }

  shell.println();

  return EXIT_SUCCESS;
}

// commands test motors
// do one rotation and measure encoder and speed
int mx_test_gear_ratio(int argc = 0, char** argv = NULL)
{
  if (argc < 2)
  {
    print_bad_arg_count();
    return -1;
  }

  // which motor
  unsigned long motor = atoi(argv[1]);

  if (motor < 1 || motor > 4)
  {
    print_index_not_in_range();
    return -1;
  }

  // how fast
  double eff = 200;
  if (argc == 3)
  {
    eff = atoi(argv[2]);
  }

  shell.print("test motor ");
  shell.println(motor);

  // zero encoder
  enc_ptr_arr[motor - 1]->zero();

  // start spinning motor
  unsigned long start_time = millis();
  emc_array[motor - 1].set_vector_effort(eff);

  // spin motor until one rotation is complete
  char inchar = shell.read();
  while(enc_ptr_arr[motor - 1]->get_pulses() < TMC_WHEEL_PPR && inchar != 'q')
  {
    emc_array[motor - 1].set_vector_effort(eff);
    shell.print("MOT:");
    shell.print(emc_array[motor - 1].get_vector_effort());
    shell.print("ENC");
    shell.print(motor);
    shell.print(":");
    shell.print(enc_ptr_arr[motor - 1]->get_log_string());
    print_append_q_quit();

    inchar = shell.read();
  }

  // calculate speed
  unsigned long time = millis() - start_time;
  double disp = enc_ptr_arr[motor - 1]->get_displacement(0, TMC_PID_PULSE_TO_POS_LIN);
  double speed = enc_ptr_arr[motor - 1]->get_velocity(disp, ((double)time / MILLIS_TO_SEC));
  double pps = (double)(enc_ptr_arr[motor - 1]->get_pulses()) / ((double)time / MILLIS_TO_SEC);

  emc_array[motor - 1].stop();

  shell.print("pps: ");
  shell.print(pps);
  shell.print(",");
  shell.print("t: ");
  shell.print((double)time / MILLIS_TO_SEC);
  shell.print(",");
  shell.print("disp: ");
  shell.print(disp);
  shell.print(",");
  shell.print("speed: ");
  shell.println(speed);

  return EXIT_SUCCESS;
}

// get top speed
int mx_test_max_effort(int argc = 0, char** argv = NULL)
{
  if (argc != 2)
  {
    print_bad_arg_count();
    return -1;
  }

  // which motor
  unsigned long motor = atoi(argv[1]);

  if (motor < 1 || motor > 4)
  {
    print_index_not_in_range();
    return -1;
  }

  shell.print("test motor ");
  shell.println(motor);

  // test max speed for 2 seconds
  double speed = emc_array[motor - 1].test_effort_response(MOTOR_EFFORT_MAX, 5000);

  // print speed
  shell.print("speed: ");
  shell.println(speed);

  return EXIT_SUCCESS;
}

// get speed to effort scalar
int mx_tune_effort_scalar(int argc = 0, char** argv = NULL)
{
  if (argc != 2)
  {
    print_bad_arg_count();
    return -1;
  }

  // which motor
  unsigned long motor = atoi(argv[1]);

  if (motor < 1 || motor > 4)
  {
    print_index_not_in_range();
    return -1;
  }

  shell.print("tune motor ");
  shell.println(motor);

  // test max speed for 2 seconds
  double veleff = emc_array[motor - 1].tune_effort_scalar(MOTOR_EFFORT_MAX);

  // print speed
  shell.print("ratio: ");
  shell.println(veleff);

  return EXIT_SUCCESS;
}

// motion control tests
int smc_drive_test(int argc = 0, char** argv = NULL)
{
  char inchar = shell.read();
  while(inchar != 'q')
  {
    shell.print(smc.get_log_string());
    print_append_q_quit();

    inchar = shell.read();

    double effort = 60;

    if (inchar == 'w')
    {
      smc.skid_drive(effort);
    }
    else if (inchar == 'a')
    {
      smc.skid_turn(effort);
    }
    else if (inchar == 'd')
    {
      smc.skid_turn(-effort);
    }
    else if (inchar == 's')
    {
      smc.skid_drive(-effort);
    }

    delay(20UL);
    smc.passive_loop(20UL);
  }

  return EXIT_SUCCESS;
}

// pid tuning
// helpers
void pid_increment_parameter(double& parameter, float increment, bool positive)
{
  if (positive)
  {
    parameter += increment;
    return;
  }
  parameter -= increment;
}

void print_pid_parameters(uint8_t index, bool positive)
{
  if(positive)
  {
    shell.print("+:");
  }
  else
  {
    shell.print("-:");
  }
  shell.print(emc_array[index].get_pid_params_string());
  shell.print(",");
  shell.print("vel:");
  shell.print(emc_array[index].get_velocity());
  shell.print(",");
  shell.print("pos:");
  shell.print(emc_array[index].get_position());
  print_append_q_quit();
}

// position
int pid_pos_tune(int argc = 0, char** argv = NULL)
{
  if (argc != 2)
  {
    print_bad_arg_count();
    return -1;
  }

  // which motor
  unsigned long motor = atoi(argv[1]);

  if (motor < 1 || motor > 4)
  {
    print_index_not_in_range();
    return -1;
  }

  emc_array[motor - 1].set_position(0.6);

  char inchar = shell.read();
  bool up = true;
  double pi_inc = 0.1;
  double d_inc = 0.001;
  while(inchar != 'q')
  {
    print_pid_parameters(motor - 1, up);

    inchar = shell.read();

    if (inchar == 'p')
    {
      pid_increment_parameter(*(emc_array[motor - 1].pkp), pi_inc, up);
    }
    else if (inchar == 'i')
    {
      pid_increment_parameter(*(emc_array[motor - 1].pki), pi_inc, up);
    }
    else if (inchar == 'd')
    {
      pid_increment_parameter(*(emc_array[motor - 1].pkd), d_inc, up);
    }
    else if (inchar == '+')
    {
      up = true;
    }
    else if (inchar == '-')
    {
      up = false;
    }
    else if (inchar == ' ')
    {
      emc_array[motor - 1].set_position(0.6);
    }

    delay(dt);
    emc_array[motor - 1].update();
  }

  return EXIT_SUCCESS;
}

// velocity
int pid_vel_tune(int argc = 0, char** argv = NULL)
{
  if (argc != 2)
  {
    print_bad_arg_count();
    return -1;
  }

  // which motor
  unsigned long motor = atoi(argv[1]);

  if (motor < 1 || motor > 4)
  {
    print_index_not_in_range();
    return -1;
  }

  emc_array[motor - 1].set_velocity(0.6);

  char inchar = shell.read();
  bool up = true;
  double pi_inc = 0.1;
  double d_inc = 0.001;
  while(inchar != 'q')
  {
    print_pid_parameters(motor - 1, up);

    inchar = shell.read();

    if (inchar == 'p')
    {
      pid_increment_parameter(*(emc_array[motor - 1].vkp), pi_inc, up);
    }
    else if (inchar == 'i')
    {
      pid_increment_parameter(*(emc_array[motor - 1].vki), pi_inc, up);
    }
    else if (inchar == 'd')
    {
      pid_increment_parameter(*(emc_array[motor - 1].vkd), d_inc, up);
    }
    else if (inchar == '+')
    {
      up = true;
    }
    else if (inchar == '-')
    {
      up = false;
    }

    delay(dt);
    emc_array[motor - 1].update();
  }

  return EXIT_SUCCESS;
}

// network
int fb_over_wire(int argc = 0, char** argv = NULL)
{
  char inchar = shell.read();
  while(inchar != 'q')
  {
    shell.print(interface_c.get_wire_string());
    // shell.println(smc.get_log_string());
    print_append_q_quit();

    delay(dt);
    smc.passive_loop(dt);

    inchar = shell.read();
  }

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
  // read battery
  console.register_config(F("batt"), console_print_battery);
  // debug wire
  console.register_config(F("netstr"), fb_over_wire);
  // set comms config
  // set loop rate and other 'constants'
  console.register_config(F("rate <loop_rate>"), set_loop_hz);
  // set pid variables

  // run special control commands
  // set e-stop
  console.register_config(F("estop <val>"), set_estop);
  // encoder tests
  console.register_config(F("encxread <enc_id>"), run_enc_test_loop);
  console.register_config(F("mxencrev <motor_id> <eff>"), mx_test_gear_ratio);
  // motor controls
  console.register_config(F("mxeffmax <motor_id>"), mx_test_max_effort);
  console.register_config(F("mxefftune <motor_id>"), mx_tune_effort_scalar);
  // motion tests
  console.register_config(F("trackdrive"), smc_drive_test);
  // pid tuning
  console.register_config(F("pidpos <motor_id>"), pid_pos_tune);
  console.register_config(F("pidvel <motor_id>"), pid_vel_tune);

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
  // TMC_SERIAL_CONFIG.println("---   ###   ---");
  print_version(&TMC_SERIAL_CONFIG, fname);
  // TMC_SERIAL_CONFIG.println("---   ###   ---");
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
