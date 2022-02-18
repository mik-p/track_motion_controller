#pragma once

#include "motor_hal.h"

#include <Sabertooth.h>

namespace tmc
{

/**
 * @brief Sabertooth packet serial motor HAL
 *
 */
class SaberToothHAL : public MotorHAL
{
#define STHAL_MOTOR_DIRECTION_FORWARD 1
#define STHAL_MOTOR_DIRECTION_REVERSE (-1)

#define STHAL_MOTOR_EFFORT_MAX 127

#define STHAL_COMMAND_TIMEOUT_DEFAULT 500  // 500ms
#define STHAL_SERIALPORT_DEFAULT SabertoothTXPinSerial

#define STHAL_M1_INDEX 1
#define STHAL_M2_INDEX 2

public:
  typedef struct
  {
    uint8_t tx_pin;
    uint8_t e_stop_pin;
  } motor_pin_map_t;

  typedef struct
  {
    uint8_t st_address;
    uint8_t st_motor_side;
  } st_driver_params_t;

  static bool ST_AUTO_BAUD_COMPLETE;

public:
  SaberToothHAL(const motor_pin_map_t& pin_map, const st_driver_params_t& st_driver_params, HardwareSerial& st_port,
                const uint32_t& st_command_timeout = STHAL_COMMAND_TIMEOUT_DEFAULT)
    : _pin_map(pin_map)
    , _st_driver_params(st_driver_params)
    , _st_command_timeout(st_command_timeout)
    , _st_driver_class(st_driver_params.st_address, (SabertoothStream&)st_port)
  {
    // init serial port and set baud
    if (!ST_AUTO_BAUD_COMPLETE)
    {
      st_port.begin(9600);
      _st_driver_class.autobaud();
      // set flag don't do this again
      SaberToothHAL::ST_AUTO_BAUD_COMPLETE = true;
    }
  }

  void init()
  {
    // stop motors
    _st_driver_class.stop();

    // estop pin
    pinMode(_pin_map.e_stop_pin, OUTPUT);
    digitalWrite(_pin_map.e_stop_pin, LOW);

    // set command timeout
    _st_driver_class.setTimeout(_st_command_timeout);

    // estop off
    digitalWrite(_pin_map.e_stop_pin, HIGH);
  }

  void set_effort(const uint8_t& new_effort)
  {
    // set current effort tracking
    _current_effort = map(new_effort, MOTOR_EFFORT_MIN, MOTOR_EFFORT_MAX, MOTOR_EFFORT_MIN, STHAL_MOTOR_EFFORT_MAX);

    // set motor to selected effort
    _st_driver_class.motor(_st_driver_params.st_motor_side, _current_effort);
  }

  void set_direction(const MOTOR_DIRECTION& new_dir)
  {
    // filter input to logic values
    int8_t dir;
    if (new_dir <= MotorHAL::MOTOR_DIRECTION::FORWARD)
    {
      dir = STHAL_MOTOR_DIRECTION_FORWARD;
    }
    else
    {
      dir = STHAL_MOTOR_DIRECTION_REVERSE;
    }

    // set the motor provided direction
    _st_driver_class.motor(_st_driver_params.st_motor_side, dir * _current_effort);
  }

protected:
  // motor hw params
  motor_pin_map_t _pin_map;

  // motor tracking variables
  uint8_t _current_effort;
  uint8_t _current_dir;

  // st driver
  uint32_t _st_command_timeout;
  st_driver_params_t _st_driver_params;
  Sabertooth _st_driver_class;
};

bool SaberToothHAL::ST_AUTO_BAUD_COMPLETE = false;
}  // namespace tmc
