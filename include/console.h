#pragma once

#include <Arduino.h>

#include <SimpleSerialShell.h>

namespace tmc
{

/**
 * @brief console emulator to interact with configuration options for controller
 *
 */
class Console
{
public:
  Console(USBSerial* ser, unsigned int baud) : _console((Stream*)ser), _baud(baud)
  {
  }

  Console(HardwareSerial* ser, unsigned int baud) : _console((Stream*)ser), _baud(baud)
  {
  }

  void init(USBSerial* ser)
  {
    ser->begin(_baud);
    shell.attach(*_console);
  }

  void init(HardwareSerial* ser)
  {
    ser->begin(_baud);
    shell.attach(*_console);
  }

  /**
   * @brief register a config options and callback
   *
   */
  void register_config(const __FlashStringHelper* name, SimpleSerialShell::CommandFunction f)
  {
    shell.addCommand(name, f);
  }

  /**
   * @brief loop the console and operate on provided options
   *
   */
  void loop()
  {
    // execute shell command if any
    shell.executeIfInput();
  }

private:
  Stream* _console;
  unsigned int _baud;
};

}  // namespace tmc
