
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

void MotorController::set_vector_effort(double new_effort, double max_effort)
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
  // scale value and send pwm
  map(new_effort, 0, max_effort, MOTOR_EFFORT_MIN, MOTOR_EFFORT_MAX);
  set_effort((uint8_t)new_effort);
}

// EncodedMotorController::EncodedMotorController() :
// {}
