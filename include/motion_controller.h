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

  uint16_t passive_loop(const uint32_t& loop_time_ms)
  {
    // unsigned long start_time = millis();
    unsigned long start_time_micros = micros();

    // update control
    passive_update(loop_time_ms);

    // update feedback and send
    update_feedback_msg();
    send_joint_feedback();

    // measure time
    // the loop time will actually be sent in the next loop
    _loop_time = millis() - _loop_start_time;
    _loop_start_time = millis();  // reset timer
    // loop time in micro seconds just measures the execution of this section of code
    _loop_time_micros = micros() - start_time_micros;

    return _loop_time;
  }

  uint16_t loop()
  {
    unsigned long start_time_micros = micros();

    // get command
    _last_command_time = assign_latest_joint_commands();

    if (_last_command_time + (unsigned long)_command_timeout > millis())
    {
      _is_timed_out = false;
    }
    else
    {
      halt();
      _is_timed_out = true;
    }

    // update control
    update();

    // send feedback
    send_joint_feedback();

    // measure time
    // the loop time will actually be sent in the next loop
    _loop_time = millis() - _loop_start_time;
    _loop_start_time = millis();  // reset timer
    // loop time in micro seconds just measures the execution of this section of code
    _loop_time_micros = micros() - start_time_micros;

    return _loop_time;
  }

  void halt()
  {
    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      _joint_array_ptr[i].halt();
    }
  }

  void set_update_interval(const unsigned long& update_interval)
  {
    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      _joint_array_ptr[i].set_update_interval(update_interval);
    }
  }

  // logging
  const String get_log_string()
  {
    String log = "";

    // fill a csv string with metrics
    // time
    log += "TIME:";
    log += millis();
    log += ",";
    log += _loop_time;
    log += ",";
    log += _loop_time_micros;
    log += ",";
    log += _last_command_time;
    log += ",";
    log += _is_timed_out;
    log += ",";
    // joints
    log += "JOINT:";
    log += _joint_array_length;
    log += ",";
    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      log += _joint_array_ptr[i].get_position();
      log += ",";
      log += _joint_array_ptr[i].get_velocity();
      log += ",";
    }
    // comms
    log += "COMM:";
    log += _interface_ptr->get_log_string();

    // return log string
    return log;
  }

protected:
  MotionController() : _joint_array_length(0), _command_timeout(1000)
  {
    // starting time for loop timing
    _loop_start_time = millis();
  }

  virtual void passive_update(const unsigned long& delta_t_ms)
  {
    double dt = (double)delta_t_ms / MILLIS_TO_SEC;
    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      _joint_array_ptr[i].passive_update(dt);
    }
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
    // get the latest message data
    interface_msg_t* msg = _interface_ptr->get_control_msg_ptr();

    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      // apply corrections
      double vel = cap_velocity(msg->data[i]);

      // assign joint command
      _joint_array_ptr[i].set_velocity(vel);
    }

    // if the message was new then update the command time
    if (_interface_ptr->accept_new_control_message())
    {
      _last_command_time = millis();
    }

    // return the current command time
    return _last_command_time;
  }

  virtual void update_feedback_msg()
  {
    // set feedback
    _interface_ptr->set_feedback_msg_header(_loop_time);

    for (uint8_t i = 0; i < _joint_array_length; ++i)
    {
      // XXX TODO apply corrections
      _interface_ptr->push_feedback_data(_joint_array_ptr[i].get_velocity());
    }
  }

  virtual void send_joint_feedback()
  {
    // set feedback
    update_feedback_msg();

    // send it and prep new command
    _interface_ptr->send_and_receive();
  }

  const double cap_velocity(const double& vel)
  {
    if (vel > TMC_PID_MAX_VEL)
    {
      return TMC_PID_MAX_VEL;
    }

    if (vel < (-1) * TMC_PID_MAX_VEL)
    {
      return (-1) * TMC_PID_MAX_VEL;
    }

    return vel;
  }

protected:
  // array control
  uint8_t _joint_array_length;

  // motion components
  EncodedMotorController* _joint_array_ptr;
  double* _command_array_ptr;

  // timing measurements - in milliseconds
  unsigned long _loop_start_time;
  uint16_t _loop_time;
  uint16_t _loop_time_micros;
  unsigned long _last_command_time;
  uint16_t _command_timeout;

  // logging measurements
  bool _is_timed_out;

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

  void skid_drive(const double& eff)
  {
    rhs_effort(eff);
    lhs_effort(eff);
  }

  void skid_turn(const double& eff)
  {
    lhs_effort(-eff);
    rhs_effort(eff);
  }

protected:
  void rhs_effort(const double& eff)
  {
    for (int i = 0; i < _joint_array_length / 2; ++i)
    {
      _joint_array_ptr[2 * i].set_vector_effort(eff);
    }
  }

  void lhs_effort(const double& eff)
  {
    for (int i = 0; i < _joint_array_length / 2; ++i)
    {
      _joint_array_ptr[2 * i + 1].set_vector_effort(eff);
    }
  }
};
}  // namespace tmc
