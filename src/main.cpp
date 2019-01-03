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

#include <Arduino.h>

#include "motor_controller.h"

// instantiate config and motor class
encoder_pin_map_t e_pin_map = {2, 4};
motor_pin_map_t m_pin_map = {7, 8, 9, DUAL_DIRECTION_PIN};
double rpm_scalar = 1;
double pulse_to_pos = 1.0 / 30.0;
unsigned long dt = 100; // ms
pid_parameters_t pos_pid = {0.5, 0.1, 0.2, 0, 0, 0, 0};
pid_parameters_t vel_pid = {0.5, 0.1, 0.2, 0, 0, 0, 0};

static encoded_motor_parameters_t params = {
  {7, 8, 9, DUAL_DIRECTION_PIN},
  {2, 4},
  1.0,
  1.0 / 30.0,
  100,
  {0.5, 0.1, 0.2, 0, 0, 0, 0},
  {0.5, 0.1, 0.2, 0, 0, 0, 0}
};

static EncodedMotorController em(&params);

double setpoint = 10;

void ENC_ISR()
{
  em.encoder_tick();
}

void setup()
{
  Serial.begin(115200);

  Serial.println("encoded motor controller test");

  pinMode(3, OUTPUT); // for testing with incremental encoder
  digitalWrite(3, LOW);

  em.init();

  em.set_encoder_interrupt(INT0, ENC_ISR);

  Serial.print("set control point: ");
  Serial.println(setpoint);
  em.set_position(setpoint);
  // em.set_velocity(setpoint);

  Serial.print("current pos / vel: ");
  Serial.print(em.get_position());
  Serial.print(" / ");
  Serial.println(em.get_velocity());

  uint16_t del = 2000;

  delay(del);

  Serial.println("begin motion");
}

void loop()
{
  em.update(); // run control loop

  // view result
  Serial.print(em.get_position());
  Serial.print(", ");
  Serial.print(em.get_velocity());
  Serial.print(", ");
  Serial.println(em.get_vector_effort(rpm_scalar * MOTOR_EFFORT_MAX));

  delay(100);

  // while(!Serial.available()) {} // wait for verification test
  // while(Serial.available()) {Serial.read();}

  // exit(0);
}
