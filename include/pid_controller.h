
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
} pid_parameters_t;


class PIDController
{
public:
  PIDController(pid_parameters_t * params);
  double pid_factory(double sig_cur, double sig_set, double delta_t);

private:
  pid_parameters_t * _pid_params;
};

#endif
