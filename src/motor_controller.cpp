
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
    _effort = new_effort;
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

EncodedMotorController::EncodedMotorController(encoded_motor_parameters_t motor_params)
  : _motor_params(motor_params), _last_update(0), _control_mode(CONTROL_MODE::POSITION)
{
  // create the shared memory for the PIDs
  _position_controller = &_motor_params.pos_pid_params;
  _velocity_controller = &_motor_params.vel_pid_params;

  // public access to params
  pkp = &_position_controller->kp;
  pki = &_position_controller->ki;
  pkd = &_position_controller->kd;
  vkp = &_velocity_controller->kp;
  vki = &_velocity_controller->ki;
  vkd = &_velocity_controller->kd;
}

EncodedMotorController::~EncodedMotorController()
{
}

void EncodedMotorController::passive_update(const double& delta_t)
{
  double displacement;
  // double delta_t = (double)delta_t_ms / MILLIS_TO_SEC;  // convert time to sec

  // read encoder and update pos/vel
  _last_update = millis();  // reset timer

  displacement = _encoder_ptr->get_displacement(PIDController::measurement(_position_controller), _motor_params.pulse_to_pos);

  PIDController::add_measurement(_position_controller, displacement);  // add to position measurement

  PIDController::measurement(_velocity_controller, _encoder_ptr->get_velocity(displacement, delta_t));  // set current velocity
}

void EncodedMotorController::update()
{
  double vel_effort, displacement;                      // effort to motor
  unsigned long delta_t_ms = millis() - _last_update;   // get measurement duration
  double delta_t = (double)delta_t_ms / MILLIS_TO_SEC;  // convert time to sec

  // if the update time has not arrived yet then wait
  if (delta_t_ms < _motor_params.update_interval)
  {
    return;
  }

  // read encoder and update pos/vel
  passive_update(delta_t);

  // determine if currently in position or velocity control
  if (_control_mode == CONTROL_MODE::POSITION)  // position control
  {
    // determine velocity setpoint based on position pid factory
    PIDController::setpoint(_velocity_controller, PIDController::improved_pid_factory(_position_controller, delta_t));
    // vel_effort = _position_controller->improved_pid_factory(delta_t);
  }

  // determine motor velocity effort based on velocity pid factory
  vel_effort = PIDController::improved_pid_factory(_velocity_controller, delta_t);

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
    _motor_ptr->set_direction(MotorHAL::MOTOR_DIRECTION::FORWARD);
    _motor_ptr->set_effort(effort);
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

  // get inverse ratio (vel/eff) because the great chance of divide by zero
  return speed / (double)effort;
}

void EncodedMotorController::halt()
{
  set_velocity(0);
}

void EncodedMotorController::set_position(const double& pos)  // unit radians
{
  _control_mode = CONTROL_MODE::POSITION;  // set mode

  PIDController::setpoint(_position_controller, pos);  // new position setpoint

  PIDController::measurement(_position_controller, 0);  // start position profile

  _encoder_ptr->zero();  // reset encoder count

  _last_update = millis();  // reset timer
}

void EncodedMotorController::set_velocity(const double& vel)  // unit rad/s
{
  // set_position(0);  // zero position control and measurements

  _control_mode = CONTROL_MODE::VELOCITY;  // set to velocity mode

  PIDController::setpoint(_velocity_controller, vel);  // new velocity setpoint
}
}  // namespace tmc
