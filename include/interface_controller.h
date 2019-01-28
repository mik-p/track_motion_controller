
#ifndef INTERFACE_CONTROLLER_H
#define INTERFACE_CONTROLLER_H

#include <HardwareSerial.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define INTERFACE_MSG_STATUS_DONE 0
#define INTERFACE_MSG_STATUS_PENDING 1

#define INTERFACE_MAX_BUFFER_SIZE 128

#define INTERFACE_DEFAULT_SERIAL_BAUD 115200
#define INTERFACE_MAX_SERIAL_BUFFER_SIZE 128

#define INTERFACE_DEFAULT_CIP_UDP_TCP_PORT 44818
#define INTERFACE_DEFAULT_CIP_UDP_IO_PORT 2222


typedef struct
{
  uint8_t msg_status;
} interface_msg_t;

typedef struct
{
  uint32_t baud_rate;
  HardwareSerial * serial_port;
} serial_interface_parameters_t;

typedef struct
{
  uint8_t spi_cs;
  byte mac[];
  IPAddress ip_address;
  unsigned int port;
} udp_interface_parameters_t;


class InterfaceController
{
  // define messsage interface to controller
  // usage:
  // get control messages for use and set feedback
  // call send_and_receive regularly to check interface traffic

public:
  interface_msg_t get_control_msg() {return _control_msg;}
  void set_feedback_msg(interface_msg_t fm){_feedback_msg = fm;}
  int send_and_receive();
  uint16_t serialise(char * buf, uint16_t len, interface_msg_t * msg);
  interface_msg_t deserialise(char * buf, uint16_t len);
  void printf(const char * fmt, ...);

protected:
  virtual void _print(char * buf);
  virtual int _receive_control_msg();
  virtual void _send_feedback_msg(char * buf, uint16_t len);

  interface_msg_t _control_msg, _feedback_msg;
};

class SerialInterfaceController : public InterfaceController
{
  // define read and write for serial interface

public:
  SerialInterfaceController(serial_interface_parameters_t * params);
  void begin() {_serial_params->serial_port->begin(_serial_params->baud_rate);}
  // void show_buffer() {_print(_buffer);}

protected:
  virtual void _print(char * buf) {_serial_params->serial_port->print(buf);}
  virtual int _receive_control_msg();
  virtual void _send_feedback_msg(char * buf, uint16_t len);

private:
  serial_interface_parameters_t * _serial_params;
  char _buffer[INTERFACE_MAX_SERIAL_BUFFER_SIZE];
  uint16_t _buffer_index;
};

class UDPInterfaceController : public InterfaceController
{
  // define read and write for udp interface

public:
  UDPInterfaceController(udp_interface_parameters_t * params);
  void begin();

private:
  // void _print(const char * buf);
  // int _receive_control_msg();
  // void _send_feedback_msg();

  udp_interface_parameters_t * _udp_params;
  EthernetUDP _udp;
  char _packet_buffer[UDP_TX_PACKET_MAX_SIZE];
};

// class HTTPInterfaceController : public InterfaceController
// {
//
// }

#endif
