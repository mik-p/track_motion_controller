
#include <Arduino.h>

#include "utils/version.h"
#include "utils/blink.h"

#include "hw_defs.h"

using namespace tmc;

// make the motion controller
SkidMotionController smc;

/**
 * @brief arduinos setup function
 * opportunity to get the hardware initialized
 *
 */
void setup()
{
  // attach hardware classes to motor control interfaces
  emc_array[0].init(&l_motor, &l_encoder);
  emc_array[1].init(&r_motor, &r_encoder);

  // set encoder interrupts
  l_encoder.set_tick_interrupt(INT0, L_ENC_ISR);
  l_encoder.set_tick_interrupt(INT2, R_ENC_ISR);

  // setup the motion controller's interface references
  smc.attach_hw_refs(&interface_c, emc_array, 2);

  // print version
  print_version(&TMC_SERIAL_CONFIG);

  // start up delay
  uint16_t del = TMC_STARTUP_DELAY;

  blink(del);
}

/**
 * @brief main control loop
 * LED blinks each time the loop passes for the duration of
 * the remaining time if the blink does not occur
 * it means that the loop over-ran
 *
 *
 */
void loop()
{
  // run motion control at full speed
  unsigned long start_time = millis();

  smc.loop();

  // delay the remaining time
  blink((dt - (millis() - start_time) / 2));
}
