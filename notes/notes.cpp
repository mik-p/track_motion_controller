// // link to decent firmware https://github.com/hbrobotics/ros_arduino_bridge/tree/indigo-devel/ros_arduino_firmware/src/libraries
// // link to ros side controller http://wiki.ros.org/diff_drive_controller
// // link to rosserial udp example https://github.com/ros-drivers/rosserial/blob/dd76994c67c5e4997ef64837c07afb4eb0d4df27/rosserial_arduino/src/ros_lib/examples/TcpHelloWorld/TcpHelloWorld.ino#L15
// // arduino PID lib https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp
//
//
// // try to make this the only arduino dependent File
//
// #include <Arduino.h>
//
// // instantiate motion controller class
// // make the motion controller class non arduino specific implementation
//
// void setup()
// {
//   Serial.begin(115200);
//   // set all control variables initial value
//   // maybe control variables from file also
//   // start network connection
//   // setup webserver for control parameters
//   // handshake control regime unless already setup
//   // save settings to File
//   // check initial diagnostics (battery etc)
// }
//
// void loop()
// {
//   // check diagnostics
//   // get control commands
//   // convert control commands to motion profile
//   // update PID model
//   // update motion commands
//   // publish motion and diagnostics
// }
