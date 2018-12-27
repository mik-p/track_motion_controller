
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

EncodedMotorController::EncodedMotorController(MotorController motor, Encoder enc, drive_parameters_t * params) :
MotorController(motor),
_encoder(enc),
_drive_params(params),
_last_update(0),
_position_setpoint(0),
_velocity_setpoint(0),
_position_current(0),
_velocity_current(0),
_position_controller(&_drive_params->pos_pid_params),
_velocity_controller(&_drive_params->vel_pid_params)
{}

void EncodedMotorController::update()
{
  double effort, displacement; // effort to motor
  unsigned long delta_t_ms = millis() - _last_update; // get measurement duration
  unsigned long delta_t = delta_t_ms / MILLIS_TO_SEC; // convert time to sec

  if(delta_t_ms >= _drive_params->update_interval) // read encoder and update pos/vel
  {
    _last_update = millis(); // reset timer

    displacement = _encoder.get_displacement(_position_current, _drive_params->pulse_to_pos);

    _position_current += displacement;

    _velocity_current = _encoder.get_velocity(displacement, delta_t);
  }
  else
  {
    return;
  }

  // determine if currently in position or velocity control
  if(_position_setpoint != 0) // position control
  {
    // determine velocity setpoint based on position pid factory
    _velocity_setpoint = _position_controller.pid_factory(_position_current, _position_setpoint, delta_t);
  }

  // determine motor effort based on velocity pid factory
  effort = _velocity_controller.pid_factory(_velocity_setpoint, _velocity_current, delta_t);

  // set motor effort (convert from rps to rpm)
  set_vector_effort(effort * RPS_TO_RPM, _drive_params->max_rpm);
}

void EncodedMotorController::tune_max_rpm()
{
  stop(); // stop motor

  _encoder.zero(); // zero encoder count

  // set motor to max speed
  set_direction(MOTOR_DIRECTION_FORWARD);
  set_effort(MOTOR_EFFORT_MAX);

  // wait to read encoder
  unsigned long t = millis();
  while(millis() - t < 10 * _drive_params->update_interval) {}

  // read encoder
  double speed = _encoder.get_velocity(
    _encoder.get_displacement(0, _drive_params->pulse_to_pos),
    10 * _drive_params->update_interval
  );

  stop(); // stop motor

  // compare with effort speed prediction
  double diff = abs((speed * RPS_TO_RPM) - _drive_params->max_rpm);

  if(diff > _drive_params->max_rpm * 0.1)
  {
    _drive_params->max_rpm = speed; // set if out of tolerance
  }
}

void EncodedMotorController::halt()
{
  set_velocity(0);
}

void EncodedMotorController::set_position(double pos) // unit radians
{
  _position_setpoint = pos; // new position setpoint

  _position_current = 0; // start position profile

  _encoder.zero(); // reset encoder count

  _last_update = millis(); // reset timer
}

void EncodedMotorController::set_velocity(double vel) // unit rad/s
{
  if(_position_setpoint != 0)
  {
    set_position(0); // disable position control
  }

  _velocity_setpoint = vel; // new velocity setpoint

}
