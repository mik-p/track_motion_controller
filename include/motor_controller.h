
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "encoder.h"
#include "pid_controller.h"


#define RPS_TO_RPM 9.5493
#define RPM_TO_RPS
#define MILLIS_TO_SEC 1000

#define SINGLE_DIRECTION_PIN 0
#define DUAL_DIRECTION_PIN 1

#define MOTOR_DIRECTION_FORWARD 0
#define MOTOR_DIRECTION_REVERSE 1

#define MOTOR_EFFORT_MIN 0
#define MOTOR_EFFORT_MAX 255

#define MOTOR_POSITIVE_DIR (1)
#define MOTOR_NEGATIVE_DIR (-1)


typedef struct
{
  uint8_t effort_pin;
  uint8_t direction_pin;
  uint8_t direction_pin_reverse;
  uint8_t DIRECTION_OPTION;
} motor_pin_map_t;

typedef struct
{
  motor_pin_map_t m_pin_map;
  encoder_pin_map_t e_pin_map;
  double rpm_scalar; // map effort to speed domain
  double pulse_to_pos; // encoder pulse to radian conversion
  unsigned long update_interval; // millisecond update interval (ideally less than 100 ms)
  pid_parameters_t pos_pid_params;
  pid_parameters_t vel_pid_params;
} encoded_motor_parameters_t;


class MotorController
{
public:
  MotorController(motor_pin_map_t * pin_map);
  void init();
  void set_effort(uint8_t new_effort);
  void set_direction(uint8_t new_dir);
  void stop();
  void set_vector_effort(double new_effort, double max_effort_rpm);
  double get_vector_effort(double max_effort_rpm);

protected:
  motor_pin_map_t * _pin_map;
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
  EncodedMotorController(encoded_motor_parameters_t * motor_params);
  void init() {MotorController::init(); _encoder.init();}
  void update();
  void tune_rpm_scalar();
  void halt();
  void set_position(double pos); // start new vel or pos control, this resets status variables
  void set_velocity(double vel);
  double get_position() {return _position_controller.measurement();} // get status of current vel or pos control
  double get_velocity() {return _velocity_controller.measurement();}

private:
  encoded_motor_parameters_t * _motor_params; // specific drive configuration
  Encoder _encoder;
  unsigned long _last_update;
  PIDController _position_controller, _velocity_controller; // velocity pid controller
};

#endif
