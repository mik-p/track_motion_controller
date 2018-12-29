// link to decent firmware https://github.com/hbrobotics/ros_arduino_bridge/tree/indigo-devel/ros_arduino_firmware/src/libraries
// link to ros side controller http://wiki.ros.org/diff_drive_controller
// link to rosserial udp example https://github.com/ros-drivers/rosserial/blob/dd76994c67c5e4997ef64837c07afb4eb0d4df27/rosserial_arduino/src/ros_lib/examples/TcpHelloWorld/TcpHelloWorld.ino#L15
// arduino PID lib https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp


// try to make this the only arduino dependent File

#include <Arduino.h>

#include "motor_controller.h"

motor_pin_map_t pin_map = {6, 4, 7, DUAL_DIRECTION_PIN};

MotorController m(&pin_map); // instantiate motor controller

// instantiate motion controller class
// make the motion controller class non arduino specific implementation

void setup()
{
  Serial.begin(115200);

  Serial.println("motor driver test");

  delay(5000);

  m.init(); // test init
  Serial.print("init: ");
  Serial.print(digitalRead(pin_map.effort_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map.direction_pin_reverse));
  Serial.println();

  delay(1000);

  m.set_direction(MOTOR_DIRECTION_FORWARD); // test set direction
  Serial.print("direction forward: ");
  Serial.print(digitalRead(pin_map.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map.direction_pin_reverse));
  Serial.println();

  while(!Serial.available()) {} // wait for electrical test

  delay(500);

  m.set_direction(MOTOR_DIRECTION_REVERSE);
  Serial.print("direction reverse: ");
  Serial.print(digitalRead(pin_map.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map.direction_pin_reverse));
  Serial.println();

  while(!Serial.available()) {} // wait for electrical test

  delay(1000);

  m.set_effort(MOTOR_EFFORT_MAX); // test set effort
  while(!Serial.available()) {} // wait for electrical test

  delay(1000);

  // set all control variables initial value
  // maybe control variables from file also
  // start network connection
  // setup webserver for control parameters
  // handshake control regime unless already setup
  // save settings to File
  // check initial diagnostics (battery etc)
}

void loop()
{


  // check diagnostics
  // get control commands
  // convert control commands to motion profile
  // update PID model
  // update motion commands
  // publish motion and diagnostics
}
