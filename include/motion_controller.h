
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "motor_controller.h"
// #include "interface_controller.h"

// #include <Arduino.h>


// typedef struct
// {
//   encoded_motor_parameters_t motor_params;
//   // transmission etc
// } wheel_joint_parameters_t;

typedef struct
{
  encoded_motor_parameters_t left_wheel;
  encoded_motor_parameters_t right_wheel;
} skid_motion_parameters_t;

typedef struct
{
  double right;
} skid_motion_status_t;

typedef struct
{
  double left_vel;
  double right_vel;
} skid_motion_msg_t;

typedef struct
{
  unsigned long supervisor_interval; // timeout for unmanaged commands
  skid_motion_msg_t msg; // actual data
} skid_motion_packet_t;


class MotionController
{
public:
  void get_commands();
  void send_feedback();

protected:
  // SerialInterfaceController _serial_interface;
  // UDPInterfaceController _udp_interface;
};

class SkidMotionController : public MotionController
{
public:
  SkidMotionController(skid_motion_parameters_t * params);
  void init() {_left_motor.init(); _right_motor.init();}
  void set_encoder_interrupts(uint8_t int_l, uint8_t int_r, void (*tick_isr)());
  void encoder_tick() {_left_motor.encoder_tick(); _right_motor.encoder_tick();}
  void update() {_left_motor.update(); _right_motor.update();}
  void halt() {_left_motor.halt(); _right_motor.halt();}
  void set_motion(skid_motion_msg_t cmd);
  skid_motion_msg_t get_feedback();
  void get_status();

private:
  skid_motion_parameters_t * _motion_params;
  EncodedMotorController _left_motor, _right_motor;
  // double _position, _velocity; // vectors in x-y plane
};

#endif
