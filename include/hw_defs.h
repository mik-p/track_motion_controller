#pragma once

#include <Arduino.h>

// XXX ALWAYS FIRST
#include "boards/pin_defs.h"

#include "motion_controller.h"

namespace tmc
{

#define TMC_STARTUP_DELAY 2000

// common params
const double vel_to_effort = 3.1;
const double pulse_to_pos = 0.0184;
const unsigned long dt = 50;  // ms

// motor control intrinsics
PIDController::pid_parameters_t pos_pid = { 0.3, 0.3, 0.1, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort) };
PIDController::pid_parameters_t vel_pid = { 0.3, 0.1, 0.05, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort) };

// collect all the connected hardware
Encoder l_encoder({ 21, 19 });
Encoder r_encoder({ 20, 22 });

// XXX TODO replace with macros??
void L_ENC_ISR()
{
  l_encoder.tick();
}

void R_ENC_ISR()
{
  r_encoder.tick();
}

#if defined(TMC_E407)
// begin config port
TMC_SERIAL_CONFIG.begin(9600);

// motors via sabre hal
SabreToothHAL l_motor();
SabreToothHAL r_motor();

// create an array of motor controllers
EncodedMotorController emc_array[]{ EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }),
                                    EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }) };

// make an interface controller
UDPInterfaceController interface_c();

#else
// motors via aad hal
AnalogAndDirMotor l_motor({ 5, 3, 2, AnalogAndDirMotor::AAD_DIR_PIN_OPT::DUAL });
AnalogAndDirMotor r_motor({ 6, 4, 7, AnalogAndDirMotor::AAD_DIR_PIN_OPT::DUAL });

// create an array of motor controllers
EncodedMotorController emc_array[]{ EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }),
                                    EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }) };

// make an interface controller
SerialInterfaceController interface_c(&TMC_SERIAL_CONTROL, 9600);
#endif
}
