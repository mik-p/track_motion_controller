#pragma once

#include <Arduino.h>

namespace tmc
{
class Battery
{
public:
    Battery(uint8_t pin, uint32_t ref, uint32_t div) : _analog_pin(pin), _reference(ref), _divider(div)
    {
        pinMode(_analog_pin, INPUT);
    }

    const float read()
    {
        uint32_t value = 0;
        float batt_volts = 0;
        float ref_div_ratio = (float)(_reference * _divider) / 10000.0;

        for (uint8_t i = 0; i < 32; i++)
        {
            value += analogRead(_analog_pin);
        }

        value >>= 5;
        batt_volts = ((float)value / 1023.0) * ref_div_ratio;

        return batt_volts;
    }

private:
    uint8_t _analog_pin;
    uint32_t _reference;
    uint32_t _divider;
};
}  // namespace tmc
