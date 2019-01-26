
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

void MotorController::set_vector_effort(double new_effort)
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

  // cap value and send pwm
  if(new_effort > MOTOR_EFFORT_MAX) set_effort(MOTOR_EFFORT_MAX);
  else if(new_effort < MOTOR_EFFORT_MIN) set_effort(MOTOR_EFFORT_MIN);
  else set_effort((uint8_t)new_effort);
}

double MotorController::get_vector_effort()
{
  if(_direction == MOTOR_DIRECTION_FORWARD)
  {
    return MOTOR_POSITIVE_DIR * _effort;
  }
  else
  {
    return MOTOR_NEGATIVE_DIR * _effort;
  }
}

EncodedMotorController::EncodedMotorController(encoded_motor_parameters_t * motor_params) :
MotorController(&motor_params->m_pin_map),
_motor_params(motor_params),
_encoder(&motor_params->e_pin_map),
_last_update(0),
_control_mode(MOTOR_POSITION_MODE),
_position_controller(&motor_params->pos_pid_params),
_velocity_controller(&motor_params->vel_pid_params)
{}

void EncodedMotorController::update()
{
  double vel_effort, displacement; // effort to motor
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
  if(_control_mode == MOTOR_POSITION_MODE) // position control
  {
    // determine velocity setpoint based on position pid factory
    _velocity_controller.setpoint(_position_controller.improved_pid_factory(delta_t));
    // vel_effort = _position_controller.improved_pid_factory(delta_t);
  }

  // determine motor velocity effort based on velocity pid factory
  vel_effort = _velocity_controller.improved_pid_factory(delta_t);

  // check limits and tolerances
  vel_effort *= _motor_params->vel_to_effort;
  

  // set motor effort (convert from rps to pwm)
  // set_vector_effort(vel_effort * _motor_params->vel_to_effort);
  set_vector_effort(vel_effort);
}

double EncodedMotorController::test_effort_response(uint8_t effort, uint32_t sample_time_ms)
{
  stop(); // stop motor

  _encoder.zero(); // zero encoder count

  // set motor to max speed
  set_direction(MOTOR_DIRECTION_FORWARD);
  set_effort(effort);

  // wait to read encoder
  unsigned long t = millis();
  while(millis() - t < sample_time_ms) {}

  // read encoder
  double disp = _encoder.get_displacement(0, _motor_params->pulse_to_pos);
  double dt = (double)sample_time_ms / MILLIS_TO_SEC;
  double speed = _encoder.get_velocity(disp, dt);

  stop(); // stop motor

  return speed;
}

double EncodedMotorController::tune_effort_scalar(uint8_t effort)
{
  double speed = test_effort_response(effort, 2000);

  return speed / (double)effort; // get inverse ratio (vel/eff)
}

void EncodedMotorController::halt()
{
  set_velocity(0);
}

void EncodedMotorController::set_position(double pos) // unit radians
{
  _control_mode = MOTOR_POSITION_MODE; // set mode

  _position_controller.setpoint(pos); // new position setpoint

  _position_controller.measurement(0); // start position profile

  _encoder.zero(); // reset encoder count

  _last_update = millis(); // reset timer
}

void EncodedMotorController::set_velocity(double vel) // unit rad/s
{
  set_position(0); // zero position control and measurements

  _control_mode = MOTOR_VELOCITY_MODE; // set to velocity mode

  _velocity_controller.setpoint(vel); // new velocity setpoint

}
