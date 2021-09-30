#pragma once

#include <Arduino.h>

namespace tmc
{
/**
 * @brief abstract layer for controlling the physical motor interface
 * this allows the rest of the library to operate the same with many differnet motors
 *
 */
class MotorHAL
{
#define MOTOR_EFFORT_MIN 0
#define MOTOR_EFFORT_MAX 255

public:
  enum MOTOR_DIRECTION
  {
    INIT,
    FORWARD,
    REVERSE
  };

public:
  ~MotorHAL() = default;

  virtual void init() = 0;
  virtual void set_effort(const uint8_t& new_effort) = 0;
  virtual void set_direction(const MOTOR_DIRECTION& new_dir) = 0;

protected:
  MotorHAL() = default;
};

/**
 * @brief a commonly used analog effort and digital direction interface
 *
 */
class AnalogAndDirMotor : public MotorHAL
{
#define AAD_MOTOR_DIRECTION_FORWARD 1
#define AAD_MOTOR_DIRECTION_REVERSE 0

#define AAD_MOTOR_EFFORT_MIN (MOTOR_EFFORT_MIN)
#define AAD_MOTOR_EFFORT_MAX (MOTOR_EFFORT_MAX)

public:
  enum AAD_DIR_PIN_OPT
  {
    SINGLE,
    DUAL
  };

  typedef struct
  {
    uint8_t effort_pin;
    uint8_t direction_pin;
    uint8_t direction_pin_reverse;
    AAD_DIR_PIN_OPT DIRECTION_OPTION;
  } motor_pin_map_t;

public:
  AnalogAndDirMotor(const motor_pin_map_t& pin_map) : _pin_map(pin_map)
  {
  }

  void init()
  {
    // set pin directions and initial zero out
    pinMode(_pin_map.effort_pin, OUTPUT);
    pinMode(_pin_map.direction_pin, OUTPUT);
    if (_pin_map.DIRECTION_OPTION == AAD_DIR_PIN_OPT::DUAL)
    {
      pinMode(_pin_map.direction_pin_reverse, OUTPUT);
    }

    digitalWrite(_pin_map.effort_pin, AAD_MOTOR_EFFORT_MIN);
  }

  void set_direction(const MOTOR_DIRECTION& new_dir)
  {
    // filter input to logic values
    uint8_t dir;
    if (new_dir >= MotorHAL::MOTOR_DIRECTION::FORWARD)
    {
      dir = AAD_MOTOR_DIRECTION_FORWARD;
    }
    else
    {
      dir = AAD_MOTOR_DIRECTION_REVERSE;
    }

    digitalWrite(_pin_map.direction_pin, dir);
    if (_pin_map.DIRECTION_OPTION == AAD_DIR_PIN_OPT::DUAL)
    {
      digitalWrite(_pin_map.direction_pin_reverse, !dir);
    }
  }

  void set_effort(const uint8_t& new_effort)
  {
    analogWrite(_pin_map.effort_pin, new_effort);
  }

protected:
  motor_pin_map_t _pin_map;
};

/**
 * @brief sabretooth packet serial motor HAL
 *
 */
class SabreToothHAL : public MotorHAL
{
public:
  typedef struct
  {
    uint8_t tx_pin;
  } motor_pin_map_t;

public:
  SabreToothHAL(const motor_pin_map_t& pin_map) : _pin_map(pin_map)
  {
  }

  void init()
  {
  }

  void set_effort(const uint8_t& new_effort)
  {
  }

  void set_direction(const uint8_t& new_dir)
  {
  }

protected:
  motor_pin_map_t _pin_map;
};
}  // namespace tmc
