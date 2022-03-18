#pragma once

#include "boards/pin_defs.h"

#include "encoder.h"

namespace tmc
{
// collect all the connected hardware
Encoder l_encoder({ L_ENC_PIN_0, L_ENC_PIN_1 });
Encoder r_encoder({ R_ENC_PIN_0, R_ENC_PIN_1 });
Encoder lb_encoder({ LB_ENC_PIN_0, LB_ENC_PIN_1 });
Encoder rb_encoder({ RB_ENC_PIN_0, RB_ENC_PIN_1 });

// XXX TODO: replace with macros??
void L_ENC_ISR()
{
  l_encoder.tick();
}

void R_ENC_ISR()
{
  r_encoder.tick();
}

#if defined(TMC_E407)
void LB_ENC_ISR()
{
  lb_encoder.tick();
}

void RB_ENC_ISR()
{
  rb_encoder.tick();
}

#endif
// set encoder interrupts
void attach_encoder_interrupts()
{
#if defined(TMC_E407)
  l_encoder.set_tick_interrupt(digitalPinToInterrupt(L_ENC_PIN_INT), L_ENC_ISR);
  r_encoder.set_tick_interrupt(digitalPinToInterrupt(R_ENC_PIN_INT), R_ENC_ISR);
  lb_encoder.set_tick_interrupt(digitalPinToInterrupt(LB_ENC_PIN_INT), LB_ENC_ISR);
  rb_encoder.set_tick_interrupt(digitalPinToInterrupt(RB_ENC_PIN_INT), RB_ENC_ISR);
#else
  l_encoder.set_tick_interrupt(L_ENC_PIN_INT, L_ENC_ISR);
  r_encoder.set_tick_interrupt(R_ENC_PIN_INT, R_ENC_ISR);
#endif

}

}  // namespace tmc
