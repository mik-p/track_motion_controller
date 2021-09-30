#pragma once

#include "motor_hal.h"
#include "encoder.h"
#include "pid_controller.h"

namespace tmc
{
// #define RPS_TO_RPM 9.5493
#define MILLIS_TO_SEC 1000

#define MOTOR_POSITIVE_DIR (1)
#define MOTOR_NEGATIVE_DIR (-1)

class MotorController
{
  // TODO implement variable limits

public:
  MotorController();
  void init(MotorHAL* motor_ptr);
  void stop();
  void set_vector_effort(const double& new_effort);
  const double get_vector_effort();

protected:
  MotorHAL* _motor_ptr;
  uint8_t _effort;
  MotorHAL::MOTOR_DIRECTION _direction;
};

class EncodedMotorController : public MotorController
{
public:
  enum CONTROL_MODE
  {
    POSITION,
    VELOCITY
  };

  typedef struct
  {
    uint8_t min_effort;
    uint8_t start_effort;
  } motor_intrinsics_t;

  typedef struct
  {
    // motor_intrinsics_t motor_intrinsics;
    double vel_to_effort;           // map effort from speed domain
    double pulse_to_pos;            // encoder pulse to radian conversion
    unsigned long update_interval;  // millisecond update interval (ideally less than 100 ms)
    PIDController::pid_parameters_t pos_pid_params;
    PIDController::pid_parameters_t vel_pid_params;
  } encoded_motor_parameters_t;

  // usage:
  // set and get velocity or position are relative control commands
  // this means that there must be a defined start and end to these commands
  // this must be managed by a motion control mechanism higher up in abstraction

  // position zero is special, it is used to switch position control off

public:
  EncodedMotorController(const encoded_motor_parameters_t& motor_params);
  ~EncodedMotorController();

  void init(MotorHAL* motor_ptr, Encoder* encoder_ptr)
  {
    MotorController::init(motor_ptr);
    _encoder_ptr = encoder_ptr;
    _encoder_ptr->init();
  }

  void update();
  const double test_effort_response(const uint8_t& effort, const uint32_t& sample_time_ms);
  const double tune_effort_scalar(const uint8_t& effort);
  void halt();
  void set_position(const double& pos);  // start new vel or pos control, this resets status variables
  void set_velocity(const double& vel);

  // get status of current vel or pos control
  const double get_position()
  {
    return _position_controller->measurement();
  }

  const double get_velocity()
  {
    return _velocity_controller->measurement();
  }

private:
  // encoder hardware ptr
  Encoder* _encoder_ptr;

  // specific drive configuration
  encoded_motor_parameters_t _motor_params;
  unsigned long _last_update;
  CONTROL_MODE _control_mode;

  // pos/vel controllers
  PIDController* _position_controller;
  PIDController* _velocity_controller;
};
}  // namespace tmc
