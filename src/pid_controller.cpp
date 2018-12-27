
#include "pid_controller.h"

#include <Arduino.h>


PIDController::PIDController(pid_parameters_t * params) :
_pid_params(params)
{}

double PIDController::pid_factory(double sig_cur, double sig_set, double delta_t)
{
  double sig_error = sig_set - sig_cur; // get error function

  double difference = (sig_error - _pid_params->previous_error) / delta_t; // get running difference

  _pid_params->previous_error = sig_error; // update running difference

  _pid_params->sum_error += sig_error * delta_t; // update running sum

  return _pid_params->kp * sig_error + _pid_params->ki * _pid_params->sum_error + _pid_params->kd * difference; // calculate output
}
