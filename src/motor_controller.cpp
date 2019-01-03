
#include "motor_controller.h"

#include <Arduino.h>


MotorController::MotorController(motor_pin_map_t * pin_map) :
_pin_map(pin_map),
_effort(MOTOR_EFFORT_MIN),
_direction(MOTOR_DIRECTION_INIT)
{}

void MotorController::init()
{
  // set pin directions and initial zero out
  pinMode(_pin_map->effort_pin, OUTPUT);
  pinMode(_pin_map->direction_pin, OUTPUT);
  if(_pin_map->DIRECTION_OPTION == DUAL_DIRECTION_PIN)
  {
    pinMode(_pin_map->direction_pin_reverse, OUTPUT);
  }

  digitalWrite(_pin_map->effort_pin, MOTOR_EFFORT_MIN);
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
    digitalWrite(_pin_map->direction_pin, _direction);
    if(_pin_map->DIRECTION_OPTION == DUAL_DIRECTION_PIN)
    {
      digitalWrite(_pin_map->direction_pin_reverse, !_direction);
    }
  }
}

void MotorController::set_effort(uint8_t new_effort)
{
  if(new_effort != _effort)
  {
    _effort = new_effort;
    analogWrite(_pin_map->effort_pin, _effort);
  }
}

void MotorController::stop()
{
  digitalWrite(_pin_map->effort_pin, MOTOR_EFFORT_MIN);
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
  new_effort = map(new_effort, 0, max_effort_rpm, MOTOR_EFFORT_MIN, MOTOR_EFFORT_MAX);

  set_effort((uint8_t)new_effort);
}

double MotorController::get_vector_effort(double max_effort_rpm)
{
  double effort = _effort;

  // map to speed domain
  effort = map(effort, MOTOR_EFFORT_MIN, MOTOR_EFFORT_MAX, 0, max_effort_rpm);

  if(_direction == MOTOR_DIRECTION_FORWARD)
  {
    return MOTOR_POSITIVE_DIR * effort;
  }
  else
  {
    return MOTOR_NEGATIVE_DIR * effort;
  }
}

EncodedMotorController::EncodedMotorController(encoded_motor_parameters_t * motor_params) :
MotorController(&_motor_params->m_pin_map),
_motor_params(motor_params),
_encoder(&_motor_params->e_pin_map),
_last_update(0),
_position_controller(&_motor_params->pos_pid_params),
_velocity_controller(&_motor_params->vel_pid_params)
{}

void EncodedMotorController::update()
{
  double effort, displacement; // effort to motor
  unsigned long delta_t_ms = millis() - _last_update; // get measurement duration
  double delta_t = (double)delta_t_ms / MILLIS_TO_SEC; // convert time to sec

  if(delta_t_ms >= _motor_params->update_interval) // read encoder and update pos/vel
  {
    _last_update = millis(); // reset timer

    displacement = _encoder.get_displacement(_position_controller.measurement(), _motor_params->pulse_to_pos);

    _position_controller.add_measurement(displacement); // add to position measurement

    _velocity_controller.measurement(_encoder.get_velocity(displacement, delta_t)); // set current velocity
  }
  else
  {
    return;
  }

  // determine if currently in position or velocity control
  if(_position_controller.setpoint() != 0) // position control
  {
    // determine velocity setpoint based on position pid factory
    _velocity_controller.setpoint(_position_controller.pid_factory(delta_t));
  }

  // determine motor effort based on velocity pid factory
  effort = _velocity_controller.pid_factory(delta_t);

  // set motor effort (convert from rps to rpm)
  set_vector_effort(effort * RPS_TO_RPM, _motor_params->rpm_scalar);
}

void EncodedMotorController::tune_rpm_scalar()
{
  stop(); // stop motor

  _encoder.zero(); // zero encoder count

  // set motor to max speed
  set_direction(MOTOR_DIRECTION_FORWARD);
  set_effort(MOTOR_EFFORT_MAX);

  // wait to read encoder
  unsigned long t = millis();
  while(millis() - t < 10 * _motor_params->update_interval) {}

  // read encoder
  double speed = _encoder.get_velocity(
    _encoder.get_displacement(0, _motor_params->pulse_to_pos),
    10 * _motor_params->update_interval
  );

  stop(); // stop motor

  // compare with effort speed prediction
  double diff = abs((speed * RPS_TO_RPM) - _motor_params->rpm_scalar);

  if(diff > _motor_params->rpm_scalar * 0.1)
  {
    _motor_params->rpm_scalar = speed; // set if out of tolerance
  }
}

void EncodedMotorController::halt()
{
  set_velocity(0);
}

void EncodedMotorController::set_position(double pos) // unit radians
{
  _position_controller.setpoint(pos); // new position setpoint

  _position_controller.measurement(0); // start position profile

  _encoder.zero(); // reset encoder count

  _last_update = millis(); // reset timer
}

void EncodedMotorController::set_velocity(double vel) // unit rad/s
{
  if(_position_controller.setpoint() != 0)
  {
    set_position(0); // disable position control
  }

  _velocity_controller.setpoint(vel); // new velocity setpoint

}
