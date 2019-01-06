
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>


typedef struct
{
  double kp;
  double ki;
  double kd;
  double sum_error;
  double previous_error;
  double signal_setpoint;
  double signal_measured;
  double signal_limit_lower;
  double signal_limit_upper;
  // double setpoint_tolerance;
} pid_parameters_t;


class PIDController
{
public:
  PIDController(pid_parameters_t * params);
  void set_upper_limit(double limit) {_pid_params->signal_limit_upper = limit;}
  void set_lower_limit(double limit) {_pid_params->signal_limit_lower = limit;}
  void zero();
  double setpoint() {return _pid_params->signal_setpoint;}
  void setpoint(double setpoint) {_pid_params->signal_setpoint = setpoint;}
  double measurement() {return _pid_params->signal_measured;}
  void measurement(double measurement) {_pid_params->signal_measured = measurement;}
  void add_measurement(double measurement) {_pid_params->signal_measured += measurement;}
  double pid_factory(double delta_t); // uses referenced pid parameters for sig_cur/set
  double pid_factory(double sig_cur, double sig_set, double delta_t);
  double improved_pid_factory(double delta_t); // PID extensions

private:
  pid_parameters_t * _pid_params;
};

#endif
