#pragma once

#include "motor_controller.h"
#include "interface_controller.h"

namespace tmc
{
/**
 * @brief basic whole body motion control
 * contains a series of joints and a control interface
 * updates each joint with commanded controls and sends feedback
 * this is achieved by a generic update loop
 *
 */
class MotionController
{
  // XXX TODO: implement both position and velocity modes

public:
  // typedef struct
  // {
  //   encoded_motor_parameters_t motor_params;
  //   // transmission etc
  // } wheel_joint_parameters_t;

public:
  ~MotionController()
  {
  }

  virtual void attach_hw_refs(InterfaceController* interface_ptr, EncodedMotorController* joint_object_array,
                              const uint8_t& joint_array_length)
  {
    _joint_array_length = joint_array_length;
    _joint_array_ptr = joint_object_array;
    _interface_ptr = interface_ptr;
  }

  void loop()
  {
    unsigned long start_time = millis();

    _last_command_time = assign_latest_joint_commands();

    if (_last_command_time + _command_timeout > millis())
    {
      update();
    }
    else
    {
      halt();
    }

    send_joint_feedback();

    // measure time
    // the loop time will actually be sent in the next loop
    _loop_time = millis() - start_time;
  }

  void halt()
  {
    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      _joint_array_ptr[i].halt();
    }
  }

protected:
  MotionController() : _joint_array_length(0), _command_timeout(1000)
  {
  }

  virtual void update()
  {
    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      _joint_array_ptr[i].update();
    }
  }

  virtual unsigned int assign_latest_joint_commands()
  {
    interface_msg_t* msg = _interface_ptr->get_control_msg_ptr();

    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      // XXX TODO apply corrections
      _joint_array_ptr[i].set_velocity(msg->data[i]);
    }

    return millis();
  }

  virtual void send_joint_feedback()
  {
    // set feedback
    // length = _joint_array_length;

    _interface_ptr->set_feedback_msg_header(_loop_time);

    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      // XXX TODO apply corrections
      _interface_ptr->push_feedback_data(_joint_array_ptr[i].get_velocity());
    }

    // send it and prep new command
    _interface_ptr->send_and_receive();
  }

protected:
  // array control
  uint8_t _joint_array_length;

  // motion components
  EncodedMotorController* _joint_array_ptr;
  double* _command_array_ptr;

  // timing measurements - in milliseconds
  uint16_t _loop_time;
  unsigned long _last_command_time;
  uint16_t _command_timeout;

  // command interface
  InterfaceController* _interface_ptr;
};

/**
 * @brief extends the basic motion controller with skid steer control corrections
 * these include:
 *  correcting slipping wheel encoder readings
 *  applying imu based yaw corrections
 *
 */
class SkidMotionController : public MotionController
{
public:
  typedef struct
  {
    double left_vel;
    double right_vel;
  } skid_motion_parameters_t;

public:
  SkidMotionController() : MotionController()
  {
  }
};
}  // namespace tmc
