#pragma once

#include <Arduino.h>

// XXX ALWAYS FIRST
#include "boards/pin_defs.h"

#include "motor_controller.h"
#include "interface_controller.h"

#if defined(TMC_E407)
#include "sabertooth_hal.h"
#endif

namespace tmc
{

#define TMC_STARTUP_DELAY 2000
#define TMC_EFFORT_RANGE 255

#if defined(TMC_E407)
#define TMC_STHAL_ADDRESS_1 128
#define TMC_STHAL_ADDRESS_2 130
#else
#endif

// common params
const double vel_to_effort = 3.1;
const double pulse_to_pos = 0.0184;
const unsigned long dt = 50;  // ms

// motor control intrinsics
PIDController::pid_parameters_t pos_pid = {
  0.3, 0.3, 0.1, 0, 0, 0, 0, -(TMC_EFFORT_RANGE / vel_to_effort), (TMC_EFFORT_RANGE / vel_to_effort)
};
PIDController::pid_parameters_t vel_pid = {
  0.3, 0.1, 0.05, 0, 0, 0, 0, -(TMC_EFFORT_RANGE / vel_to_effort), (TMC_EFFORT_RANGE / vel_to_effort)
};

// collect all the connected hardware
Encoder l_encoder({ L_ENC_PIN_0, L_ENC_PIN_1 });
Encoder r_encoder({ R_ENC_PIN_0, R_ENC_PIN_1 });

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
// motors via Saber hal
SaberToothHAL l_motor({ L_MOTOR_PIN_0, L_MOTOR_PIN_1 }, { TMC_STHAL_ADDRESS_1, STHAL_M1_INDEX },
                      STHAL_SERIALPORT_DEFAULT);
SaberToothHAL r_motor({ R_MOTOR_PIN_0, R_MOTOR_PIN_1 }, { TMC_STHAL_ADDRESS_1, STHAL_M2_INDEX },
                      STHAL_SERIALPORT_DEFAULT);

// create an array of motor controllers
EncodedMotorController emc_array[]{ EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }),
                                    EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }) };

// make an interface controller
SerialInterfaceController interface_c(&TMC_SERIAL_CONTROL, 115200);
// UDPInterfaceController interface_c();

#else
// motors via aad hal
AnalogAndDirMotor l_motor({ L_MOTOR_PIN_0, L_MOTOR_PIN_1, L_MOTOR_PIN_2, AnalogAndDirMotor::AAD_DIR_PIN_OPT::DUAL });
AnalogAndDirMotor r_motor({ R_MOTOR_PIN_0, R_MOTOR_PIN_1, R_MOTOR_PIN_2, AnalogAndDirMotor::AAD_DIR_PIN_OPT::DUAL });

// create an array of motor controllers
EncodedMotorController emc_array[]{ EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }),
                                    EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }) };

// make an interface controller
SerialInterfaceController interface_c(&TMC_SERIAL_CONTROL, 9600);
#endif
}  // namespace tmc
