#pragma once

#include "platform/motor_defs.h"
#include "platform/interface_defs.h"

#include "battery.h"
#include "estop.h"
#include "motion_controller.h"

namespace tmc
{

// make a battery
Battery battery(BATT_PIN, BATT_REF, BATT_DIV);

// make an estop
EStop estop(E_STOP_PIN);

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
  smc.attach_battery(&battery);

  // prime estop
  estop.prime();
}

}  // namespace tmc
