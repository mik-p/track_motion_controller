
#include "encoder.h"

#include <Arduino.h>


Encoder::Encoder(uint8_t a_pin, uint8_t b_pin) :
_A_phase_pin(a_pin),
_B_phase_pin(b_pin),
_A_phase_last(LOW),
_direction(ENCODER_POSITIVE_DIR),
_encoder_pulses(0),
_speed(0)
{}

void Encoder::init()
{
  pinMode(_B_phase_pin, INPUT); // set b phase to read
}

void Encoder::tick()
{
  uint8_t A_phase_current = digitalRead(_A_phase_pin); // read new A phase

  if((_A_phase_last == LOW) && (A_phase_current == HIGH)) // add some jitter tolerance ??
  {
    uint8_t B_phase_current = digitalRead(_B_phase_pin); // read new B phase

    // update direction on change of B phase
    if((B_phase_current == LOW) && (_direction == ENCODER_POSITIVE_DIR))
    {
      _direction = ENCODER_NEGATIVE_DIR; // Reverse
    }
    else if((B_phase_current == HIGH) && (_direction == ENCODER_NEGATIVE_DIR))
    {
      _direction = ENCODER_POSITIVE_DIR;  // Forward
    }
  }

  _A_phase_last = A_phase_current; // update last A phase

  // update encoder pulse count
  if(_direction == ENCODER_POSITIVE_DIR)
  {
    _encoder_pulses++;
  }
  else
  {
    _encoder_pulses--;
  }
}

void Encoder::set_tick_interrupt(uint8_t interrupt)
{
  attachInterrupt(interrupt, tick_isr, CHANGE); // set interrupt on change of value
}

void Encoder::tick_isr()
{
  this.tick();
}
