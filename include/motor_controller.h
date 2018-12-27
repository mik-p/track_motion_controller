
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "encoder.h"


#define RPS_TO_RPM 9.5493
#define RPM_TO_RPS

#define MOTOR_DIRECTION_FORWARD 0
#define MOTOR_DIRECTION_REVERSE 1

#define MOTOR_EFFORT_MIN 0
#define MOTOR_EFFORT_MAX 255

#define MOTOR_POSITIVE_DIR (1)
#define MOTOR_NEGATIVE_DIR (-1)


class MotorController
{
public:
  MotorController(uint8_t ef_pin, uint8_t dir_pin);
  void init();
  void set_effort(uint8_t new_effort);
  void set_direction(uint8_t new_dir);
  void stop();
  void set_vector_effort(double new_effort, double max_effort_rpm);
  double get_vector_effort(double max_effort_rpm);

protected:
  uint8_t _effort_pin;
  uint8_t _direction_pin;
  uint8_t _effort;
  uint8_t _direction;
};

class EncodedMotorController : public MotorController
{
public:
  EncodedMotorController(MotorController motor, Encoder encoder, double max_rpm);
  void init() {MotorController::init(); _encoder.init();}
  void tune_max_rpm();
  void set_velocity(double rad_per_s);
  double get_velocity();

private:
  Encoder _encoder;
  double _max_rpm; // map effort to speed domain
};

#endif
