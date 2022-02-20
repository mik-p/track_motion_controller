#pragma once

#include "boards/pin_defs.h"
#include "platform/pid_defs.h"
#include "platform/encoder_defs.h"

#include "motor_controller.h"

#if defined(TMC_E407)
#include "sabertooth_hal.h"
#endif

namespace tmc
{
#if defined(TMC_E407)
#define TMC_STHAL_ADDRESS_1 128
#define TMC_STHAL_ADDRESS_2 130
#else
#endif

#if defined(TMC_E407)
// motors via Saber hal
SaberToothHAL l_motor({ L_MOTOR_PIN_0, L_MOTOR_PIN_1 },
                      { &STHAL_SERIALPORT_DEFAULT, TMC_STHAL_ADDRESS_1, STHAL_M1_INDEX });
SaberToothHAL r_motor({ R_MOTOR_PIN_0, R_MOTOR_PIN_1 },
                      { &STHAL_SERIALPORT_DEFAULT, TMC_STHAL_ADDRESS_1, STHAL_M2_INDEX });

// create an array of motor controllers
EncodedMotorController emc_array[]{ EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }),
                                    EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }) };

void setup_motors()
{
  // attach hardware classes to motor control interfaces
  emc_array[0].init(&l_motor, &l_encoder);
  emc_array[1].init(&r_motor, &r_encoder);
}

#else
// motors via aad hal
AnalogAndDirMotor l_motor({ L_MOTOR_PIN_0, L_MOTOR_PIN_1, L_MOTOR_PIN_2, AnalogAndDirMotor::AAD_DIR_PIN_OPT::DUAL });
AnalogAndDirMotor r_motor({ R_MOTOR_PIN_0, R_MOTOR_PIN_1, R_MOTOR_PIN_2, AnalogAndDirMotor::AAD_DIR_PIN_OPT::DUAL });

// create an array of motor controllers
EncodedMotorController emc_array[]{ EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }),
                                    EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }) };
#endif

}  // namespace tmc
