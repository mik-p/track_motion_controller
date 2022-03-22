#pragma once

#include <stdint.h>

namespace tmc
{
/**
 * @brief PID controller class
 * uses a shared memory to apply control corrections
 *
 */
class PIDController
{
public:
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

public:
  PIDController() = delete;
  // PIDController(pid_parameters_t* params);

  static void set_upper_limit(pid_parameters_t* _pid_params, const double& limit)
  {
    _pid_params->signal_limit_upper = limit;
  }

  static void set_lower_limit(pid_parameters_t* _pid_params, const double& limit)
  {
    _pid_params->signal_limit_lower = limit;
  }

  static void zero(pid_parameters_t* _pid_params);

  static double setpoint(pid_parameters_t* _pid_params)
  {
    return _pid_params->signal_setpoint;
  }

  static void setpoint(pid_parameters_t* _pid_params, const double& setpoint)
  {
    _pid_params->signal_setpoint = setpoint;
  }

  static double measurement(pid_parameters_t* _pid_params)
  {
    return _pid_params->signal_measured;
  }

  static void measurement(pid_parameters_t* _pid_params, const double& measurement)
  {
    _pid_params->signal_measured = measurement;
  }

  static void add_measurement(pid_parameters_t* _pid_params, const double& measurement)
  {
    _pid_params->signal_measured += measurement;
  }

  static double pid_factory(pid_parameters_t* _pid_params, const double& delta_t);  // uses referenced pid parameters for sig_cur/set
  static double pid_factory(pid_parameters_t* _pid_params, double sig_cur, double sig_set, double delta_t);
  static double improved_pid_factory(pid_parameters_t* _pid_params, const double& delta_t);  // PID extensions

// private:
  // pid_parameters_t* _pid_params;
};

}  // namespace tmc
