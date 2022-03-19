#pragma once

#include <Arduino.h>
// #include <stdint.h>

namespace tmc
{
#define ENCODER_POSITIVE_DIR (1)
#define ENCODER_NEGATIVE_DIR (-1)

void TICK_ISR();  // declare global interrupt

class Encoder
{
public:
  typedef struct
  {
    uint8_t A_phase_pin;  // pin map variables
    uint8_t B_phase_pin;
  } encoder_pin_map_t;

  // usage:
  // must pass an interrupt to attach to and specify
  // global scope method that calls this 'tick' method

public:
  Encoder(const encoder_pin_map_t& pin_map);

  void init();
  void tick();
  void set_tick_interrupt(uint8_t interrupt, void (*tick_isr)());

  void set_encoder_pulses(const int32_t& pulses)
  {
    _encoder_pulses = pulses;
  }

  const int32_t get_pulses()
  {
    return _encoder_pulses;
  }

  void zero()
  {
    set_encoder_pulses(0);
  }

  const double get_displacement(const double& pos_i, const double& pulse_to_pos);  // calculate displacement given pos
                                                                                   // and pulse to radian conversion
  const double get_velocity(const double& disp, const double& delta_t);  // calculate velocity given disp & timestep

  // debug string
  const String get_log_string()
  {
    String log = "";

    log += digitalRead(_pin_map.A_phase_pin);
    log += ",";
    log += digitalRead(_pin_map.B_phase_pin);
    log += ",";
    log += get_pulses();

    return log;
  }

private:
  encoder_pin_map_t _pin_map;
  volatile uint8_t _A_phase_last;    // last value of A phase
  volatile int8_t _direction;        // rotation direction (defined by B phase)
  volatile int32_t _encoder_pulses;  // number of encoder pulses (rolls over)
};

}  // namespace tmc
