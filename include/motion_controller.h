
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "motor_controller.h"

#include <Arduino.h>


#define SERIAL_BUFFER_SIZE 128


typedef struct
{
  uint8_t ef_pin;
  uint8_t dir_pin;
  uint8_t a_pin;
  uint8_t b_pin;
  drive_parameters_t drive_params;
  // transmission etc
} wheel_joint_parameters_t;

typedef struct
{
  wheel_joint_parameters_t left_joint;
  wheel_joint_parameters_t right_joint;
} skid_motion_parameters_t;

typedef struct
{
  double right;
} skid_motion_status_t;

typedef struct
{
  double left_pos;
  double left_vel;
  double right_pos;
  double right_vel;
} skid_motion_command_t;

typedef struct
{

} skid_motion_packet_t;


class MotionController
{
public:
  void attach_serial(Stream & serial) {_serial = &serial;}
  void get_serial_packet();

protected:
  Stream * _serial;
  uint8_t _serial_buffer[SERIAL_BUFFER_SIZE];
  uint8_t _serial_buffer_size;
};

class SkidMotionController : public MotionController
{
public:
  SkidMotionController(skid_motion_parameters_t * params);
  void init() {_left_motor.init(); _right_motor.init();}
  void update();
  void set_motion(skid_motion_command_t motion_command);

private:
  skid_motion_parameters_t * _motion_params;
  EncodedMotorController _left_motor, _right_motor;
  double _position, _velocity;
};

#endif
