
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "encoder.h"


#define MOTOR_DIRECTION_FORWARD 0
#define MOTOR_DIRECTION_REVERSE 1

#define MOTOR_EFFORT_MIN 0
#define MOTOR_EFFORT_MAX 255


class MotorController
{
public:
  MotorController(uint8_t ef_pin, uint8_t dir_pin);
  void init();
  void set_effort(uint8_t new_effort);
  void set_direction(uint8_t new_dir);
  void stop();
  void set_vector_effort(double new_effort, double max_effort);

protected:
  uint8_t _effort_pin;
  uint8_t _direction_pin;
  uint8_t _effort;
  uint8_t _direction;
};

class EncodedMotorController : public MotorController
{
public:
private:
  Encoder _encoder;
};

#endif
