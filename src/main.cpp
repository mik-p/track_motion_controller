
#include <Arduino.h>

#include "motor_controller.h"

#include <OPC.h>
#include <Ethernet.h>

////////////////////////////////////////////////////////////////////////////////

OPCEthernet opc_interface_controller;

byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xAD, 0x8D };

IPAddress ip(192, 168, 0, 10);
// IPAddress gateway(192, 168, 10, 1);
// IPAddress dns_server(192, 168, 0, 1);
// IPAddress subnet(255,255,255,0);

const int listen_port = 8888;

opcOperation digital_status_input[14], analog_status_input[6];

bool readwrite_digital(const char *itemID, const opcOperation opcOP, const bool value)
{
  byte port;

  port = atoi(&itemID[1]);

  if (opcOP == opc_opwrite) {
    if (digital_status_input[port] == opc_opread) {
      digital_status_input[port] = opc_opwrite;
      pinMode(port,OUTPUT);
    }

    digitalWrite(port,value);
  }
  else
  {
    if (digital_status_input[port] == opc_opwrite) {
      digital_status_input[port] = opc_opread;
     // pinMode(port,INPUT);
    }

    return digitalRead(port);
  }

}

int readwrite_analog(const char *itemID, const opcOperation opcOP, const int value) {
  byte port;

  port = atoi(&itemID[1]);

  if (opcOP == opc_opwrite) {
    if (analog_status_input[port] == opc_opread) {
      analog_status_input[port] = opc_opwrite;
      pinMode(port,OUTPUT);
    }

    analogWrite(port,value);
  }
  else
  {
    if (analog_status_input[port] == opc_opwrite) {
      analog_status_input[port] = opc_opread;
      //pinMode(port,INPUT);
    }

    return analogRead(port);
  }

}

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////

// common
double vel_to_effort = 3.1;
double pulse_to_pos = 0.0184;
unsigned long dt = 50; // ms
pid_parameters_t pos_pid = {0.3, 0.3, 0.1, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort)};
pid_parameters_t vel_pid = {0.3, 0.1, 0.05, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort)};

// right
motor_pin_map_t m_pins_r = {6, 4, 7, DUAL_DIRECTION_PIN}; // stop second motor
encoder_pin_map_t e_pins_r = {20, 22};

static encoded_motor_parameters_t motor_params_r = {
  m_pins_r,
  e_pins_r,
  vel_to_effort,
  pulse_to_pos,
  dt,
  pos_pid,
  vel_pid
};

static EncodedMotorController motor_r(&motor_params_r);

void ENC_ISR_R()
{
  motor_r.encoder_tick();
}

// left
motor_pin_map_t m_pins_l = {5, 3, 2, DUAL_DIRECTION_PIN};
encoder_pin_map_t e_pins_l = {21, 19};

static encoded_motor_parameters_t motor_params_l = {
  m_pins_l,
  e_pins_l,
  vel_to_effort,
  pulse_to_pos,
  dt,
  pos_pid,
  vel_pid
};

static EncodedMotorController motor_l(&motor_params_l);

void ENC_ISR_L()
{
  motor_l.encoder_tick();
}

////////////////////////////////////////////////////////////////////////////////


// double pos_setpoint = 500;
double setpoint = 0.5;

uint8_t start_effort = 60;


void setup()
{
  Serial.begin(115200);

  Serial.println("OPC interface and encoded motor controller test");

  motor_r.init();
  motor_l.init();

  motor_r.set_encoder_interrupt(INT0, ENC_ISR_R);
  motor_l.set_encoder_interrupt(INT2, ENC_ISR_L);

  uint16_t del = 2000;

  delay(del);

  Serial.print("set control point: ");
  Serial.println(setpoint);
  motor_r.set_velocity(setpoint);
  motor_l.set_velocity(setpoint);

  Serial.println("setup OPC server");

  byte k;

  for (k=0;k<14;k++) digital_status_input[k] = opc_opread;
  for (k=0;k<5;k++)  analog_status_input[k] = opc_opread;

  opc_interface_controller.setup(listen_port, mac, ip);

  opc_interface_controller.addItem("D0",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D1",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D2",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D3",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D4",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D5",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D5",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D6",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D7",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D8",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D9",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D10",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D11",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D12",opc_readwrite, opc_bool, readwrite_digital);
  opc_interface_controller.addItem("D13",opc_readwrite, opc_bool, readwrite_digital);

  opc_interface_controller.addItem("A0",opc_readwrite, opc_int, readwrite_analog);
  opc_interface_controller.addItem("A1",opc_readwrite, opc_int, readwrite_analog);
  opc_interface_controller.addItem("A2",opc_readwrite, opc_int, readwrite_analog);
  opc_interface_controller.addItem("A3",opc_readwrite, opc_int, readwrite_analog);
  opc_interface_controller.addItem("A4",opc_readwrite, opc_int, readwrite_analog);
  opc_interface_controller.addItem("A5",opc_readwrite, opc_int, readwrite_analog);

  Serial.println("begin motion");
}

void loop()
{
  opc_interface_controller.processOPCCommands();
  // char inchar = 'f';
  // while(inchar != 's')
  // {
  //   motor_r.update(); // run control loop
  //   motor_l.update(); // run control loop
  //
  //   // view result
  //   Serial.print(motor_r.get_position());
  //   Serial.print(", ");
  //   Serial.print(motor_r.get_velocity());
  //   Serial.print(", ");
  //   Serial.print(motor_r.get_vector_effort());
  //   Serial.print(" : ");
  //   Serial.print(motor_l.get_position());
  //   Serial.print(", ");
  //   Serial.print(motor_l.get_velocity());
  //   Serial.print(", ");
  //   Serial.println(motor_l.get_vector_effort());
  //
  //   delay(dt);
  //
  //   inchar = Serial.read();
  //   if(inchar == 'i')
  //   {
  //     setpoint += 0.2;
  //     motor_r.set_velocity(setpoint);
  //     Serial.println(setpoint);
  //   }
  //   else if(inchar == 'k')
  //   {
  //     setpoint -= 0.2;
  //     motor_r.set_velocity(setpoint);
  //     Serial.println(setpoint);
  //   }
  // }
  //
  // motor_r.halt();
  // motor_l.halt();

  // exit(0);
}
