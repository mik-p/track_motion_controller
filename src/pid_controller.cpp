
#include "pid_controller.h"

#include <Arduino.h>


PIDController::PIDController(pid_parameters_t * params) :
_pid_params(params)
{}

void PIDController::zero()
{
  _pid_params->sum_error = 0;
  _pid_params->previous_error = 0;
  _pid_params->signal_measured = 0;
}

double PIDController::pid_factory(double delta_t)
{
  double sig_error = _pid_params->signal_setpoint - _pid_params->signal_measured; // get error function

  double difference = (sig_error - _pid_params->previous_error) / delta_t; // get running difference

  _pid_params->previous_error = sig_error; // update running difference

  _pid_params->sum_error += sig_error * delta_t; // update running sum

  return _pid_params->kp * sig_error + _pid_params->ki * _pid_params->sum_error + _pid_params->kd * difference; // calculate output
}

double PIDController::pid_factory(double sig_cur, double sig_set, double delta_t)
{
  double sig_error = sig_set - sig_cur; // get error function

  double difference = (sig_error - _pid_params->previous_error) / delta_t; // get running difference

  _pid_params->previous_error = sig_error; // update running difference

  _pid_params->sum_error += sig_error * delta_t; // update running sum

  return _pid_params->kp * sig_error + _pid_params->ki * _pid_params->sum_error + _pid_params->kd * difference; // calculate output
}

double PIDController::improved_pid_factory(double delta_t)
{
  // removes derivative kick,
  // improves on the fly parameter change stability
  // reduce integral wind up lag
  // implements output limits

  // TODO scale parameters with constant time

  double sig_error = _pid_params->signal_setpoint - _pid_params->signal_measured; // get error function

  // remove derivative kick (kd*e -> - kd*dm/dt : dm = meas - meas_last)
  double difference = -((_pid_params->signal_measured - _pid_params->previous_error) / delta_t); // get running difference

  _pid_params->previous_error = sig_error; // update running difference

  // store sum error with control parameter pushed into sum (e_sum -> ki*e_sum)
  _pid_params->sum_error += _pid_params->ki* sig_error * delta_t; // update running sum

  // cap sum error to output limits
  if(_pid_params->sum_error > _pid_params->signal_limit_upper)
  {
    _pid_params->sum_error = _pid_params->signal_limit_upper;
  }
  else if(_pid_params->sum_error < _pid_params->signal_limit_lower)
  {
    _pid_params->sum_error = _pid_params->signal_limit_lower;
  }

  // compute output
  double output = _pid_params->kp * sig_error + _pid_params->sum_error + _pid_params->kd * difference;

  // cap output to limits
  if(output > _pid_params->signal_limit_upper)
  {
    output = _pid_params->signal_limit_upper;
  }
  else if(output < _pid_params->signal_limit_lower)
  {
    output = _pid_params->signal_limit_lower;
  }

  return output;
}
