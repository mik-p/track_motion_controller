
#include <Arduino.h>

#include "motion_controller.h"

using namespace tmc;

// common params
double vel_to_effort = 3.1;
double pulse_to_pos = 0.0184;
unsigned long dt = 50;  // ms

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

// motors via aad hal
AnalogAndDirMotor aad_l({ 5, 3, 2, AnalogAndDirMotor::AAD_DIR_PIN_OPT::DUAL });
AnalogAndDirMotor aad_r({ 6, 4, 7, AnalogAndDirMotor::AAD_DIR_PIN_OPT::DUAL });

// motor control intrinsics
PIDController::pid_parameters_t pos_pid = { 0.3, 0.3, 0.1, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort) };
PIDController::pid_parameters_t vel_pid = { 0.3, 0.1, 0.05, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort) };

// create an array of motor controllers
EncodedMotorController emc_array[]{ EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }),
                                    EncodedMotorController({ vel_to_effort, pulse_to_pos, dt, pos_pid, vel_pid }) };

// make an interface controller
SerialInterfaceController interface_c(&Serial1, 9600);

// make the motion controller
SkidMotionController smc;

// XXX TODO add LED feedback

void setup()
{
  // attach hardware classes to motor control interfaces
  emc_array[0].init(&aad_l, &l_encoder);
  emc_array[1].init(&aad_r, &r_encoder);

  // set encoder interrupts
  l_encoder.set_tick_interrupt(INT0, L_ENC_ISR);
  l_encoder.set_tick_interrupt(INT2, R_ENC_ISR);

  // setup the motion controller's interface references
  smc.attach_hw_refs(&interface_c, emc_array, 2);

  uint16_t del = 2000;

  delay(del);
}

void loop()
{
  // run motion control at full speed
  unsigned long start_time = millis();

  smc.loop();

  // delay the remaining time
  delay(dt - (millis() - start_time));
}
