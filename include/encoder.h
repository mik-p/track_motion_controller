
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>


#define ENCODER_POSITIVE_DIR (1)
#define ENCODER_NEGATIVE_DIR (-1)

void TICK_ISR(); // declare global interrupt


typedef struct
{
  uint8_t A_phase_pin; // pin map variables
  uint8_t B_phase_pin;
} encoder_pin_map_t;


class Encoder
{
  // usage:
  // must pass an interrupt to attach to and specify
  // global scope method that calls this 'tick' method

public:
  Encoder(encoder_pin_map_t * pin_map);
  void init();
  void tick();
  void set_tick_interrupt(uint8_t interrupt, void (*tick_isr)());
  void set_encoder_pulses(uint32_t pulses) {_encoder_pulses = pulses;}
  void zero() {set_encoder_pulses(0);}
  double get_displacement(double pos_i, double pulse_to_pos); // calculate displacement given pos and pulse to radian conversion
  double get_velocity(double disp, double delta_t) {return disp / delta_t;} // calculate velocity given disp & timestep

private:
  encoder_pin_map_t * _pin_map;
  volatile uint8_t _A_phase_last; // last value of A phase
  volatile int8_t _direction; // rotation direction (defined by B phase)
  volatile uint32_t _encoder_pulses; // number of encoder pulses (rolls over)
};

#endif
