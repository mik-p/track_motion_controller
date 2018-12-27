
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "encoder.h"
#include "pid_controller.h"


#define RPS_TO_RPM 9.5493
#define RPM_TO_RPS
#define MILLIS_TO_SEC 1000

#define MOTOR_DIRECTION_FORWARD 0
#define MOTOR_DIRECTION_REVERSE 1

#define MOTOR_EFFORT_MIN 0
#define MOTOR_EFFORT_MAX 255

#define MOTOR_POSITIVE_DIR (1)
#define MOTOR_NEGATIVE_DIR (-1)


typedef struct
{
  double max_rpm; // map effort to speed domain
  double max_vel; // max velocity for position control
  // double max_accel; // max acceleration for motion
  // double pos_tolerance; // position
  double pulse_to_pos; // encoder pulse to radian conversion
  unsigned long update_interval; // millisecond update interval (ideally less than 100 ms)
  // unsigned long supervisor_interval; // timeout for unmanaged commands
  pid_parameters_t pos_pid_params;
  pid_parameters_t vel_pid_params;
} drive_parameters_t;


class MotorController
{
public:
  MotorController(uint8_t ef_pin, uint8_t dir_pin);
  void init();
  void set_effort(uint8_t new_effort);
  void set_direction(uint8_t new_dir);
  void stop();
  void set_vector_effort(double new_effort, double max_effort_rpm);
  double get_vector_effort(double max_effort_rpm);

protected:
  uint8_t _effort_pin;
  uint8_t _direction_pin;
  uint8_t _effort;
  uint8_t _direction;
};

class EncodedMotorController : public MotorController
{
  // usage:
  // set and get velocity or position are relative control commands
  // this means that there must be a defined start and end to these commands
  // this must be managed by a motion control mechanism higher up in abstraction

  // position zero is special, it is used to switch position control off

public:
  EncodedMotorController(MotorController motor, Encoder encoder, drive_parameters_t * params);
  void init() {MotorController::init(); _encoder.init();}
  void update();
  void tune_max_rpm();
  void halt();
  void set_position(double pos); // start new vel or pos control, this resets status variables
  void set_velocity(double vel);
  double get_position() {return _position_current;} // get status of current vel or pos control
  double get_velocity() {return _velocity_current;}

private:
  Encoder _encoder;
  drive_parameters_t * _drive_params; // specific drive configuration
  unsigned long _last_update;
  double _position_setpoint, _velocity_setpoint; // control set points
  double _position_current, _velocity_current; // status of control
  PIDController _position_controller, _velocity_controller; // velocity pid controller
};

#endif
