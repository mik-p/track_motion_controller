#pragma once

#include <Arduino.h>

namespace tmc
{

/**
 * @brief console emulator to interact with configuration options for controller
 *
 */
class console
{
public:
    console(HardwareSerial* ser, unsigned int baud) : _console(ser), _baud(baud)
    {
    }

    void init()
    {
        _console->begin(_baud);
    }

    /**
     * @brief register a config options and callback
     *
     */
    void register_config()
    {
    }

    /**
     * @brief loop the console and operate on provided options
     *
     */
    void loop()
    {

    }

private:
    HardwareSerial* _console;
    unsigned int _baud;
};

}
