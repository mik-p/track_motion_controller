
#include <Arduino.h>

#include "motion_controller.h"

#include <Ethernet.h>
#include <aREST.h>

using namespace tmc;

////////////////////////////////////////////////////////////////////////////////

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress ip(192, 168, 1, 177);
// IPAddress gateway(192, 168, 10, 1);
// IPAddress dns_server(192, 168, 0, 1);
// IPAddress subnet(255,255,255,0);

// Ethernet server
EthernetServer server(80);

// Create aREST instance
aREST rest = aREST();

// Variables to be exposed to the API
double velocity_r;
double velocity_l;

double cmd_vel_r;
double cmd_vel_l;

// Custom function accessible by the API
int set_velocity_r(String command)
{
  // Get state from command
  cmd_vel_r = command.toFloat();
  Serial.println((double)cmd_vel_r);
  return (int)cmd_vel_r;
}

int set_velocity_l(String command)
{
  // Get state from command
  cmd_vel_l = command.toFloat();
  Serial.println((double)cmd_vel_l);
  return (int)cmd_vel_l;
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

// common params
double vel_to_effort = 3.1;
double pulse_to_pos = 0.0184;
unsigned long dt = 50;  // ms
pid_parameters_t pos_pid = { 0.3, 0.3, 0.1, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort) };
pid_parameters_t vel_pid = { 0.3, 0.1, 0.05, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort) };

// left params
motor_pin_map_t m_pins_l = { 5, 3, 2, DUAL_DIRECTION_PIN };
encoder_pin_map_t e_pins_l = { 21, 19 };

static encoded_motor_parameters_t motor_params_l = { m_pins_l, e_pins_l, vel_to_effort, pulse_to_pos,
                                                     dt,       pos_pid,  vel_pid };

// right params
motor_pin_map_t m_pins_r = { 6, 4, 7, DUAL_DIRECTION_PIN };  // stop second motor
encoder_pin_map_t e_pins_r = { 20, 22 };

static encoded_motor_parameters_t motor_params_r = { m_pins_r, e_pins_r, vel_to_effort, pulse_to_pos,
                                                     dt,       pos_pid,  vel_pid };

skid_motion_parameters_t motion_params = { motor_params_l, motor_params_r };

static SkidMotionController motion_c(&motion_params);

void ENC_ISR()
{
  motion_c.encoder_tick();
}

////////////////////////////////////////////////////////////////////////////////

// double pos_setpoint = 500;
double setpoint = 0.5;

uint8_t start_effort = 60;

void setup()
{
  Serial.begin(115200);

  Serial.println("aREST interface and motion controller test");

  motion_c.init();

  motion_c.set_encoder_interrupts(INT0, INT2, ENC_ISR);

  uint16_t del = 2000;

  delay(del);

  Serial.println("setup aREST server");

  rest.variable("vel_r", &velocity_r);
  rest.variable("vel_l", &velocity_l);

  // Function to be exposed
  rest.function("cmd_vel_r", set_velocity_r);
  rest.function("cmd_vel_l", set_velocity_l);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("007");
  rest.set_name("motion_c");

  // Start the Ethernet connection and the server
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  Serial.println("begin motion");
}

void loop()
{
  char inchar = 'f';
  while (inchar != 's')
  {
    skid_motion_msg_t msg = motion_c.get_feedback();  // update feedback
    velocity_l = msg.left_vel;
    velocity_r = msg.right_vel;

    // listen for incoming clients
    EthernetClient client = server.available();
    rest.handle(client);

    motion_c.set_motion({ cmd_vel_l, cmd_vel_r });  // update set points

    motion_c.update();  // run control loop

    // view result
    Serial.print(velocity_l);
    // Serial.print(", ");
    // Serial.print(motor_r.get_vector_effort());
    Serial.print(" : ");
    Serial.print(velocity_r);
    // Serial.print(", ");
    // Serial.println(motor_l.get_vector_effort());
    Serial.println();

    delay(dt);

    inchar = Serial.read();
  }

  motion_c.halt();

  // exit(0);
}
