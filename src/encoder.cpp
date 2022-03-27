
#include "encoder.h"

namespace tmc
{
Encoder::Encoder(const encoder_pin_map_t& pin_map)
  : _pin_map(pin_map), _A_phase_last(LOW), _direction(ENCODER_POSITIVE_DIR), _encoder_pulses(0)
{
}

void Encoder::init()
{
  // set phases to read with pullups
  pinMode(_pin_map.A_phase_pin, INPUT_PULLUP);
  pinMode(_pin_map.B_phase_pin, INPUT_PULLUP);

  zero();  // zero pulse count
}

void Encoder::tick()
{
  uint8_t A_phase_current = digitalReadFast(digitalPinToPinName(_pin_map.A_phase_pin));  // read new A phase
  uint8_t B_phase_current = digitalReadFast(digitalPinToPinName(_pin_map.B_phase_pin));  // read new B phase

  if ((B_phase_current == LOW))
  {
    _direction = ENCODER_NEGATIVE_DIR;  // Reverse
    // update encoder pulse count
    _encoder_pulses--;
  }
  else
  {
    _direction = ENCODER_POSITIVE_DIR;  // Forward
    _encoder_pulses++;
  }

  _A_phase_last = A_phase_current;  // update last A phase
  _B_phase_last = B_phase_current;  // update last B phase
}

void Encoder::set_tick_interrupt(uint8_t interrupt, void (*tick_isr)())
{
  attachInterrupt(interrupt, tick_isr, RISING);  // set interrupt on change of value
}

const double Encoder::get_displacement(const double& pos_i, const double& pulse_to_pos)
{
  double pos_f = (double)(_encoder_pulses)*pulse_to_pos;  // convert pos to pulses

  return pos_f - pos_i;  // FIXME: return radians (currently revolutions?)
}

const double Encoder::get_velocity(const double& disp, const double& delta_t)
{
  return disp / delta_t;
}
}  // namespace tmc
