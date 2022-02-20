#pragma once

#include "boards/pin_defs.h"

#include "encoder.h"

namespace tmc
{
// collect all the connected hardware
Encoder l_encoder({ L_ENC_PIN_0, L_ENC_PIN_1 });
Encoder r_encoder({ R_ENC_PIN_0, R_ENC_PIN_1 });

// XXX TODO: replace with macros??
void L_ENC_ISR()
{
  l_encoder.tick();
}

void R_ENC_ISR()
{
  r_encoder.tick();
}

// set encoder interrupts
void attach_encoder_interrupts()
{
  l_encoder.set_tick_interrupt(L_ENC_PIN_INT, L_ENC_ISR);
  r_encoder.set_tick_interrupt(R_ENC_PIN_INT, R_ENC_ISR);
}
}
