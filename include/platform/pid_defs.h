#pragma once

#include "boards/pid_params.h"

#include "pid_controller.h"

namespace tmc
{
#ifndef TMC_LOOP_HZ
#define TMC_LOOP_HZ 20
#endif

#define TMC_PID_EFFORT_RANGE 255

// common params
const double vel_to_effort = TMC_PID_VEL_TO_EFF;
const double pulse_to_pos = TMC_PID_PULSE_TO_POS;
unsigned long dt = ((double)(1.0 / TMC_LOOP_HZ) * 1000.0);  // ms

// motor control intrinsics
PIDController::pid_parameters_t pos_pid = { TMC_POS_PID_KP,
                                            TMC_POS_PID_KI,
                                            TMC_POS_PID_KD,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            -(TMC_PID_EFFORT_RANGE / vel_to_effort),
                                            (TMC_PID_EFFORT_RANGE / vel_to_effort) };
PIDController::pid_parameters_t vel_pid = { TMC_VEL_PID_KP,
                                            TMC_VEL_PID_KI,
                                            TMC_VEL_PID_KD,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            -(TMC_PID_EFFORT_RANGE / vel_to_effort),
                                            (TMC_PID_EFFORT_RANGE / vel_to_effort) };

const unsigned long get_loop_time_remaining(const unsigned long& start_time)
{
  const unsigned long run_time = millis() - start_time;
  return (dt > run_time) ? (dt - run_time) : 0;
}

}  // namespace tmc
