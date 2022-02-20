#pragma once

#include "pid_controller.h"

namespace tmc
{
#ifndef TMC_LOOP_HZ
#define TMC_LOOP_HZ 20
#endif

#define TMC_PID_EFFORT_RANGE 255

// common params
const double vel_to_effort = 3.1;
const double pulse_to_pos = 0.0184;
const unsigned long dt = ((1 / TMC_LOOP_HZ) * 1000);  // ms

// motor control intrinsics
PIDController::pid_parameters_t pos_pid = {
  0.3, 0.3, 0.1, 0, 0, 0, 0, -(TMC_PID_EFFORT_RANGE / vel_to_effort), (TMC_PID_EFFORT_RANGE / vel_to_effort)
};
PIDController::pid_parameters_t vel_pid = {
  0.3, 0.1, 0.05, 0, 0, 0, 0, -(TMC_PID_EFFORT_RANGE / vel_to_effort), (TMC_PID_EFFORT_RANGE / vel_to_effort)
};

const unsigned long get_loop_time_remaining(const unsigned long& start_time)
{
  return dt - (millis() - start_time);
}

}  // namespace tmc
