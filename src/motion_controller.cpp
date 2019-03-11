
#include "motion_controller.h"


// void MotionController::get_serial_packet()
// {
//   // if(_serial->available())
//   // {
//   //   _serial_buffer[_serial_buffer_size++] = (uint8_t)_serial->read();
//   // }
//
//   // check
// }

SkidMotionController::SkidMotionController(skid_motion_parameters_t * motion_params) :
_motion_params(motion_params),
_left_motor(&_motion_params->left_wheel),
_right_motor(&_motion_params->right_wheel)
{
  // memset(_serial_buffer, '\0', SERIAL_BUFFER_SIZE);
  // _serial_buffer_size = 0;
}

void SkidMotionController::set_encoder_interrupts(uint8_t int_l, uint8_t int_r, void (*tick_isr)())
{
  _left_motor.set_encoder_interrupt(int_l, tick_isr);
  _right_motor.set_encoder_interrupt(int_r, tick_isr);
}

// void SkidMotionController::update()
// {
//   // update each motor controller
//   _left_motor.update();
//   _right_motor.update();
// }

void SkidMotionController::set_motion(skid_motion_msg_t cmd)
{
  _left_motor.set_velocity(cmd.left_vel);
  _right_motor.set_velocity(cmd.right_vel);
}

skid_motion_msg_t SkidMotionController::get_feedback()
{
  skid_motion_msg_t feedback = {
    _left_motor.get_velocity(),
    _right_motor.get_velocity()
  };
  return feedback;
}

void get_status()
{

}
