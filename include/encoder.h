
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>


#define ENCODER_POSITIVE_DIR (1)
#define ENCODER_NEGATIVE_DIR (-1)

void TICK_ISR(); // declare global interrupt


class Encoder
{
public:
  Encoder(uint8_t a_pin, uint8_t b_pin, uint8_t inter);
  void init();
  void tick();
  void calculate_velocity();

private:
  uint8_t _A_phase_pin; // pin map variables
  uint8_t _B_phase_pin;
  uint8_t _interrupt; // interrupt number
  volatile uint8_t _A_phase_last; // last value of A phase
  volatile int8_t _direction; // rotation direction (defined by B phase)
  volatile uint32_t _encoder_pulses; // number of encoder pulses (rolls over)
  volatile double _speed; // absolute magnitude of velocity
};

#endif
