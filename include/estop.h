#pragma once

#include <Arduino.h>

namespace tmc
{
class EStop
{
public:
  EStop(uint8_t pin) : _estop_pin(pin)
  {
    pinMode(_estop_pin, OUTPUT);
  }

  void reset()
  {
    digitalWrite(_estop_pin, HIGH);
  }

  void prime()
  {
    digitalWrite(_estop_pin, LOW);
  }

  void stop()
  {
    digitalWrite(_estop_pin, LOW);
  }

  const uint8_t read()
  {
    return digitalRead(_estop_pin);
  }

private:
  uint8_t _estop_pin;
};
}  // namespace tmc
