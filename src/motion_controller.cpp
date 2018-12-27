
#include "motion_controller.h"


void MotionController::get_serial_packet()
{
  if(_serial->available())
  {
    _serial_buffer[_serial_buffer_size++] = (uint8_t)_serial->read();
  }

  // check
}

SkidMotionController::SkidMotionController(skid_motion_parameters_t * params) :
_motion_params(params),
_left_motor(
  MotorController(_motion_params->left_joint.ef_pin, _motion_params->left_joint.dir_pin),
  Encoder(_motion_params->left_joint.a_pin, _motion_params->left_joint.b_pin),
  &_motion_params->left_joint.drive_params
),
_right_motor(
  MotorController(_motion_params->right_joint.ef_pin, _motion_params->right_joint.dir_pin),
  Encoder(_motion_params->right_joint.a_pin, _motion_params->right_joint.b_pin),
  &_motion_params->right_joint.drive_params
)
{
  memset(_serial_buffer, '\0', SERIAL_BUFFER_SIZE);
  _serial_buffer_size = 0;
}

void SkidMotionController::update()
{
  // update each motor controller
  _left_motor.update();
  _right_motor.update();

  // get odometry
  _left_motor.get_velocity();
  _right_motor.get_velocity();

}

void set_motion(skid_motion_command_t)
{

}

void printMotorInfo() {
}
