#pragma once

#include "platform/motor_defs.h"
#include "platform/interface_defs.h"

#include "motion_controller.h"

namespace tmc
{

// make the motion controller
SkidMotionController smc;

void tmc_init_controller()
{
  // attach hardware classes to motor control interfaces
  setup_motors();

  // set encoder interrupts
  attach_encoder_interrupts();

  // setup the motion controller's interface and motor references
  setup_interfaces();
  smc.attach_hw_refs(&interface_c, emc_array, 4);
}

}  // namespace tmc
