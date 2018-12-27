
#include "motor_controller.h"

#include <Arduino.h>


MotorController::MotorController(uint8_t ef_pin, uint8_t dir_pin) :
_effort_pin(ef_pin),
_direction_pin(dir_pin),
_effort(MOTOR_EFFORT_MIN),
_direction(MOTOR_DIRECTION_FORWARD)
{}

void MotorController::init()
{
  // set pin directions and initial zero out
  digitalWrite(_effort_pin, MOTOR_EFFORT_MIN);
  pinMode(_effort_pin, OUTPUT);
  pinMode(_direction_pin, OUTPUT);
}

void MotorController::set_direction(uint8_t new_dir)
{
  // filter input to logic values
  if(new_dir >= MOTOR_DIRECTION_FORWARD)
  {
    new_dir = MOTOR_DIRECTION_FORWARD;
  }
  else
  {
    new_dir = MOTOR_DIRECTION_REVERSE;
  }

  if(new_dir != _direction) // update if new value
  {
    _direction = new_dir;
    digitalWrite(_direction_pin, _direction);
  }
}

void MotorController::set_effort(uint8_t new_effort)
{
  if(new_effort != _effort)
  {
    _effort = new_effort;
    analogWrite(_effort_pin, _effort);
  }
}

void MotorController::stop()
{
  digitalWrite(_effort_pin, MOTOR_EFFORT_MIN);
  _effort = MOTOR_EFFORT_MIN;
}

void MotorController::set_vector_effort(double new_effort, double max_effort_rpm)
{
  if(new_effort < 0)
  {
    new_effort = (-1) * new_effort;
    set_direction(MOTOR_DIRECTION_REVERSE);
  }
  else
  {
    set_direction(MOTOR_DIRECTION_FORWARD);
  }

  // scale value from speed domain and send pwm
  map(new_effort, 0, max_effort_rpm, MOTOR_EFFORT_MIN, MOTOR_EFFORT_MAX);
  set_effort((uint8_t)new_effort);
}

double MotorController::get_vector_effort(double max_effort_rpm)
{
  double effort = _effort;

  map(effort, MOTOR_EFFORT_MIN, MOTOR_EFFORT_MAX, 0, max_effort_rpm); // map to speed domain

  if(_direction == MOTOR_DIRECTION_FORWARD)
  {
    return MOTOR_POSITIVE_DIR * effort;
  }
  else
  {
    return MOTOR_NEGATIVE_DIR * effort;
  }
}

EncodedMotorController::EncodedMotorController(MotorController motor, Encoder enc, double max_rpm) :
MotorController(motor),
_encoder(enc),
_max_rpm(max_rpm)
{}

void EncodedMotorController::tune_max_rpm()
{
  // measure speed and compare with effort speed prediction
}

void EncodedMotorController::set_velocity(double rad_per_s)
{
  double effort_rpm = rad_per_s * RPS_TO_RPM; // convert to rpm

  set_vector_effort(effort_rpm, _max_rpm);
}

double EncodedMotorController::get_velocity()
{
  // return _encoder.
}
