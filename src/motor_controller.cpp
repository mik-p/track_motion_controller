
#include "motor_controller.h"

#include <Arduino.h>

namespace tmc
{
MotorController::MotorController() : _effort(MOTOR_EFFORT_MIN), _direction(MotorHAL::MOTOR_DIRECTION::INIT)
{
}

void MotorController::init(MotorHAL* motor_ptr)
{
  // init the motor hal
  _motor_ptr = motor_ptr;
  _motor_ptr->init();

  // set effort lowest
  _motor_ptr->set_effort(MOTOR_EFFORT_MIN);
}

void MotorController::stop()
{
  _effort = MOTOR_EFFORT_MIN;
  _motor_ptr->set_effort(_effort);
}

void MotorController::set_vector_effort(const double& new_effort)
{
  // get direction from vector sign
  if (new_effort < 0)
  {
    _effort = (-1) * new_effort;
    _direction = MotorHAL::MOTOR_DIRECTION::REVERSE;
  }
  else
  {
    _direction = MotorHAL::MOTOR_DIRECTION::FORWARD;
  }

  // cap vector value to min/max
  if (_effort > MOTOR_EFFORT_MAX)
    _effort = MOTOR_EFFORT_MAX;
  else if (_effort < MOTOR_EFFORT_MIN)
    _effort = MOTOR_EFFORT_MIN;

  // set direction
  _motor_ptr->set_direction(_direction);
  // set effort
  _motor_ptr->set_effort((uint8_t)_effort);
}

const double MotorController::get_vector_effort()
{
  if (_direction == MotorHAL::MOTOR_DIRECTION::FORWARD)
  {
    return MOTOR_POSITIVE_DIR * _effort;
  }
  else
  {
    return MOTOR_NEGATIVE_DIR * _effort;
  }
}

EncodedMotorController::EncodedMotorController(const encoded_motor_parameters_t& motor_params)
  : _motor_params(motor_params), _last_update(0), _control_mode(CONTROL_MODE::POSITION)
{
  // create the shared memory for the PIDs
  _position_controller = new PIDController(&_motor_params.pos_pid_params);
  _velocity_controller = new PIDController(&_motor_params.vel_pid_params);
}

EncodedMotorController::~EncodedMotorController()
{
  delete _position_controller;
  delete _velocity_controller;
}

void EncodedMotorController::update()
{
  double vel_effort, displacement;                      // effort to motor
  unsigned long delta_t_ms = millis() - _last_update;   // get measurement duration
  double delta_t = (double)delta_t_ms / MILLIS_TO_SEC;  // convert time to sec

  if (delta_t_ms >= _motor_params.update_interval)  // read encoder and update pos/vel
  {
    _last_update = millis();  // reset timer

    displacement = _encoder_ptr->get_displacement(_position_controller->measurement(), _motor_params.pulse_to_pos);

    _position_controller->add_measurement(displacement);  // add to position measurement

    _velocity_controller->measurement(_encoder_ptr->get_velocity(displacement, delta_t));  // set current velocity
  }
  else
  {
    return;
  }

  // determine if currently in position or velocity control
  if (_control_mode == CONTROL_MODE::POSITION)  // position control
  {
    // determine velocity setpoint based on position pid factory
    _velocity_controller->setpoint(_position_controller->improved_pid_factory(delta_t));
    // vel_effort = _position_controller->improved_pid_factory(delta_t);
  }

  // determine motor velocity effort based on velocity pid factory
  vel_effort = _velocity_controller->improved_pid_factory(delta_t);

  // check limits and tolerances
  vel_effort *= _motor_params.vel_to_effort;

  // set motor effort (convert from rps to pwm)
  // set_vector_effort(vel_effort * _motor_params.vel_to_effort);
  set_vector_effort(vel_effort);
}

const double EncodedMotorController::test_effort_response(const uint8_t& effort, const uint32_t& sample_time_ms)
{
  stop();  // stop motor

  _encoder_ptr->zero();  // zero encoder count

  // set motor to max speed
  _motor_ptr->set_direction(MotorHAL::MOTOR_DIRECTION::FORWARD);
  _motor_ptr->set_effort(effort);

  // wait to read encoder
  unsigned long t = millis();
  while (millis() - t < sample_time_ms)
  {
  }

  // read encoder
  double disp = _encoder_ptr->get_displacement(0, _motor_params.pulse_to_pos);
  double dt = (double)sample_time_ms / MILLIS_TO_SEC;
  double speed = _encoder_ptr->get_velocity(disp, dt);

  stop();  // stop motor

  return speed;
}

const double EncodedMotorController::tune_effort_scalar(const uint8_t& effort)
{
  double speed = test_effort_response(effort, 2000);

  return speed / (double)effort;  // get inverse ratio (vel/eff)
}

void EncodedMotorController::halt()
{
  set_velocity(0);
}

void EncodedMotorController::set_position(const double& pos)  // unit radians
{
  _control_mode = CONTROL_MODE::POSITION;  // set mode

  _position_controller->setpoint(pos);  // new position setpoint

  _position_controller->measurement(0);  // start position profile

  _encoder_ptr->zero();  // reset encoder count

  _last_update = millis();  // reset timer
}

void EncodedMotorController::set_velocity(const double& vel)  // unit rad/s
{
  set_position(0);  // zero position control and measurements

  _control_mode = CONTROL_MODE::VELOCITY;  // set to velocity mode

  _velocity_controller->setpoint(vel);  // new velocity setpoint
}
}  // namespace tmc
