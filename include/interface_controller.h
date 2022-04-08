#pragma once

#include <HardwareSerial.h>
// #include <SPI.h>
#ifndef TMC_E407
#include <Ethernet.h>
#else
#include <LwIP.h>
#include <STM32Ethernet.h>
#endif
#include <EthernetUdp.h>
#include <CRC.h>

namespace tmc
{
#ifndef BYTE_ORDER_UTIL_MACROS
#define BYTE_ORDER_UTIL_MACROS

#define tmc_htons(x) ((((x) << 8) & 0xFF00) | (((x) >> 8) & 0xFF))
#define tmc_ntohs(x) htons(x)

#define tmc_htonl(x)                                                                                                   \
  (((x) << 24 & 0xFF000000UL) | ((x) << 8 & 0x00FF0000UL) | ((x) >> 8 & 0x0000FF00UL) | ((x) >> 24 & 0x000000FFUL))
#define tmc_ntohl(x) htonl(x)

#endif

// defining a BIG_ENDIAN representation of our data (network byte order)
#define INTERFACE_HEADER_SIZE (sizeof(uint8_t) + (4 * sizeof(uint16_t)))
#define INTERFACE_MAX_DATA_SIZE 8
#define INTERFACE_MAX_BUFFER_SIZE (INTERFACE_HEADER_SIZE + (INTERFACE_MAX_DATA_SIZE * sizeof(float)))

#ifndef TMC_CHANNEL_LOGGING
#define TMC_CHANNEL_LOGGING 0
#endif

#if TMC_CHANNEL_LOGGING
// channel debug
void debug_print_channel(const char* buf, const uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    TMC_SERIAL_CONFIG.print(buf[i], HEX);
    TMC_SERIAL_CONFIG.print(' ');
  }
  TMC_SERIAL_CONFIG.println();
}
#endif

typedef struct
{
  uint8_t status;      // 8 bits to use for status flags currently only used as e-stop
  uint16_t loop_time;  // in milliseconds
  uint16_t batt_mv; // battery in millivolts
  // unsigned long cmd_time;   // command in controller time ms
  // unsigned long fb_time;    // feedback in controller time ms
  uint16_t sequence;  // incrementing sequence number
  uint16_t length;    // data buffer length
  float* data;        // the data
} interface_msg_t;

/**
 * @brief base class for an interface to the controller
 *
 */
class InterfaceController
{
  // define message interface to controller
  // usage:
  // get control messages for use and set feedback
  // call send_and_receive regularly to check interface traffic

public:
  ~InterfaceController()
  {
  }

  const String get_wire_string()
  {
    // create buffer
    uint16_t len = INTERFACE_MAX_BUFFER_SIZE;
    char buf[len];
    String str = "";

    // serialise the msg into the buffer
    const uint16_t send_len = _serialise(buf, len, _feedback_msg);

    for (int i = 0; i < send_len; i++)
    {
      str += (int)buf[i];
      str += ' ';
    }

    return str;
  }

  const String get_log_string()
  {
    String log = "";

    // control message
    log += _control_msg.status;
    log += ",";
    log += _control_msg.loop_time;
    log += ",";
    log += _control_msg.batt_mv;
    log += ",";
    log += _control_msg.sequence;
    log += ",";
    log += _control_msg.length;
    log += ",";
    for (uint8_t i = 0; i < _control_msg.length; ++i)
    {
      log += _control_msg.data[i];
      log += ",";
    }

    // feedback message
    log += _feedback_msg.status;
    log += ",";
    log += _feedback_msg.loop_time;
    log += ",";
    log += _feedback_msg.batt_mv;
    log += ",";
    log += _feedback_msg.sequence;
    log += ",";
    log += _feedback_msg.length;
    log += ",";
    for (uint8_t i = 0; i < _feedback_msg.length; ++i)
    {
      log += _feedback_msg.data[i];
      log += ",";
    }

    // create buffers for messages
    // uint16_t len = INTERFACE_MAX_BUFFER_SIZE;
    // char cmd_buf[len];
    // char fb_buf[len];

    // serialise the msgs into buffers and add to log
    // _serialise(cmd_buf, len, _control_msg);
    // for (uint16_t i = 0; i < len; i++)
    // {
    //   log += (int)cmd_buf[i];
    //   log += ' ';
    // }

    // log += ',';

    // _serialise(fb_buf, len, _feedback_msg);
    // for (uint16_t i = 0; i < len; i++)
    // {
    //   log += (int)fb_buf[i];
    //   log += ' ';
    // }

    return log;
  }

  interface_msg_t* get_control_msg_ptr()
  {
    return &_control_msg;
  }

  void set_feedback_msg_header(const uint16_t& loop_time, const uint16_t& batt, const uint8_t& estop)
  {
    _feedback_msg.status = !estop;
    _feedback_msg.loop_time = loop_time;
    _feedback_msg.batt_mv = batt;
    // _feedback_msg.fb_time = millis();
    _feedback_msg.length = 0;
    _feedback_msg.sequence++;             // increment sequence number
    _feedback_msg.data = _feedback_data;  // set to internal buffer

    // memset the buffer empty just in case
    memset(_feedback_msg.data, 0, INTERFACE_MAX_DATA_SIZE);
  }

  const uint16_t push_feedback_data(const float& data)
  {
    if (_feedback_msg.length < INTERFACE_MAX_DATA_SIZE)
    {
      _feedback_msg.data[_feedback_msg.length++] = data;
    }

    return _feedback_msg.length;
  }

  const uint16_t send_and_receive()
  {
    _send_feedback_msg();           // send feedback to controller
    return _receive_control_msg();  // receive message from interface
  }

  const bool accept_new_control_message()
  {
    if (_recvd_new_cntrl_msg)
    {
      // new message accepted and is no longer new
      _recvd_new_cntrl_msg = false;
      return true;
    }

    return false;
  }

protected:
  InterfaceController()
  {
    // init feedback msg
    _feedback_msg.status = 0;
    _feedback_msg.loop_time = 0;
    _feedback_msg.batt_mv = 0;
    // _feedback_msg.cmd_time = 0;
    // _feedback_msg.fb_time = 0;
    _feedback_msg.length = 0;
    _feedback_msg.sequence = 0;

    // setup the msg buffers
    _feedback_msg.data = _feedback_data;
    _control_msg.data = _control_data;

    memset(_feedback_msg.data, 0, INTERFACE_MAX_DATA_SIZE);
    memset(_control_msg.data, 0, INTERFACE_MAX_DATA_SIZE);

    // no new messages
    _recvd_new_cntrl_msg = false;
  }

  const uint16_t _serialise(char* buf, const uint16_t& len, const interface_msg_t& msg)
  {
    uint16_t result_len = 0;

    // shove the bytes and words in the following order:

    // status
    buf[result_len++] = _feedback_msg.status;

    // loop time
    buf[result_len++] = _feedback_msg.loop_time >> 8;      // high byte
    buf[result_len++] = _feedback_msg.loop_time & 0x00FF;  // low byte

    // batt volts
    buf[result_len++] = _feedback_msg.batt_mv >> 8;      // high byte
    buf[result_len++] = _feedback_msg.batt_mv & 0x00FF;  // low byte

    // sequence
    buf[result_len++] = _feedback_msg.sequence >> 8;      // high byte
    buf[result_len++] = _feedback_msg.sequence & 0x00FF;  // low byte

    // length
    buf[result_len++] = _feedback_msg.length >> 8;      // high byte
    buf[result_len++] = _feedback_msg.length & 0x00FF;  // low byte

    // data in floats
    for (uint8_t i = 0; i < _feedback_msg.length; ++i)
    {
      const long nl_dat = tmc_htonl(*(long*)&_feedback_msg.data[i]);
      memcpy((void*)&buf[result_len], (const void*)&nl_dat, sizeof(long));
      result_len += sizeof(float);
    }

    // create a crc
    uint8_t packet_crc8 = crc8((uint8_t*)buf, result_len, 0x07, 0x00, 0x00, false, false);

    // return the buffer length
    return result_len;
  }

  interface_msg_t* _deserialise(char* buf, const uint16_t& len)
  {
    uint16_t result_len = 0;

    if (len < INTERFACE_HEADER_SIZE)
    {
      _recvd_new_cntrl_msg = false;
      return &_control_msg;  // return ptr to control data
    }

    // first byte is status
    _control_msg.status = buf[result_len++];

    // second two bytes is loop time
    _control_msg.loop_time = ((uint16_t)buf[result_len] << 8) | buf[result_len + 1];
    result_len++;
    result_len++;

    // next two bytes is batt volts
    _control_msg.batt_mv = ((uint16_t)buf[result_len] << 8) | buf[result_len + 1];
    result_len++;
    result_len++;

    // third two bytes is sequence
    _control_msg.sequence = ((uint16_t)buf[result_len] << 8) | buf[result_len + 1];
    result_len++;
    result_len++;

    // four two bytes is length
    _control_msg.length = ((uint16_t)buf[result_len] << 8) | buf[result_len + 1];
    result_len++;
    result_len++;

    // check that length is acceptable size
    if (_control_msg.length > INTERFACE_MAX_DATA_SIZE)
    {
      _recvd_new_cntrl_msg = false;
      return &_control_msg;  // return ptr to control data
    }

    // the rest of data is sets of sizeof(float) length long
    for (uint8_t i = 0; i < _control_msg.length; ++i)
    {
      const long hl_dat = tmc_ntohl(*(long*)&buf[result_len]);
      memcpy((void*)&_control_msg.data[i], (const void*)&hl_dat, sizeof(float));
      result_len += sizeof(long);
    }

    // new valid message
    _recvd_new_cntrl_msg = true;
    return &_control_msg;  // return ptr to control data
  }

  const uint16_t _receive_control_msg()
  {
    // create buffer
    uint16_t len = INTERFACE_MAX_BUFFER_SIZE;
    char buf[len];

    // fill with recieve data
    const uint16_t recv_len = _recv_buffer(buf, len);

    // deserialise the received data
    interface_msg_t* msg_ptr = _deserialise(buf, recv_len);

    return msg_ptr->length;
  }

  void _send_feedback_msg()
  {
    // create buffer
    uint16_t len = INTERFACE_MAX_BUFFER_SIZE;
    char buf[len];

    // serialise the msg into the buffer
    const uint16_t send_len = _serialise(buf, len, _feedback_msg);

    // send the buffer
    _send_buffer(buf, send_len);
  }

  // interface specific read and write calls
  virtual void _send_buffer(const char* buf, const uint16_t len) = 0;
  virtual const uint16_t _recv_buffer(char* buf, const uint16_t len) = 0;

protected:
  // the data storage for the interface
  interface_msg_t _control_msg, _feedback_msg;
  float _feedback_data[INTERFACE_MAX_DATA_SIZE];
  float _control_data[INTERFACE_MAX_DATA_SIZE];
  // tracking recieved messages
  bool _recvd_new_cntrl_msg;
};

/**
 * @brief serial based interface
 *
 */
class SerialInterfaceController : public InterfaceController
{
  // define read and write for serial interface

public:
  SerialInterfaceController(HardwareSerial* ser, const unsigned int baud) : _baud(baud)
  {
    _port = ser;
  }

  void init()
  {
    _port->begin(_baud);
  }

protected:
  virtual void _send_buffer(const char* buf, const uint16_t len)
  {
    _port->write(buf, len);
    _port->println();  // print a '\n' as a control character

    // debug
#if TMC_CHANNEL_LOGGING
    debug_print_channel(buf, len);
#endif
  }

  virtual const uint16_t _recv_buffer(char* buf, const uint16_t len)
  {
    uint16_t write_len = 0;

    while (_port->available() && write_len < len)
    {
      char ch = _port->read();

      // detect end control character
      if (ch == '\n')
      {
        break;
      }

      // write serialised data to buffer
      buf[write_len++] = ch;
    }

    return write_len;
  }

private:
  HardwareSerial* _port;
  unsigned int _baud;
};

/**
 * @brief udp network based interface
 *
 */
class UDPInterfaceController : public InterfaceController
{
#define TMC_IC_UDP_INTERFACE_PORT 2021

  // define read and write for udp interface

public:
  UDPInterfaceController(uint8_t spi_cs, uint8_t* mac, IPAddress ip_address)
    : _spi_cs(spi_cs)
    , _local_mac(mac)
    , _local_ip(ip_address)
    , _remote_ip(0, 0, 0, 0)
    , _remote_port(TMC_IC_UDP_INTERFACE_PORT)
  {
  }

  void init()
  {
#ifndef TMC_E407
    Ethernet.init(_spi_cs);
#endif
    Ethernet.begin(_local_mac, _local_ip);
    _udp.begin(TMC_IC_UDP_INTERFACE_PORT);
  }

protected:
  virtual void _send_buffer(const char* buf, const uint16_t len)
  {
    int check = _udp.beginPacket(_remote_ip, _remote_port);
    _udp.write(buf, len);
    _udp.endPacket();

    // debug
#if TMC_CHANNEL_LOGGING
    debug_print_channel(buf, len);
#endif
  }

  virtual const uint16_t _recv_buffer(char* buf, const uint16_t len)
  {
    uint16_t write_len = 0;
    char pckt_buf[UDP_TX_PACKET_MAX_SIZE];

    // check port
    int pckt_size = _udp.parsePacket();

    // if message
    if (pckt_size)
    {
      // get sender
      _remote_ip = _udp.remoteIP();
      _remote_port = _udp.remotePort();

      // read the packet
      _udp.read(pckt_buf, UDP_TX_PACKET_MAX_SIZE);

      while (write_len < len)
      {
        char ch = pckt_buf[write_len];

        // detect end control character
        if (ch == '\n')
        {
          break;
        }

        // write serialised data to buffer
        buf[write_len++] = ch;
      }
    }

    return write_len;
  }

private:
  EthernetUDP _udp;

  // local
  uint8_t _spi_cs;
  uint8_t* _local_mac;
  IPAddress _local_ip;

  // remote
  IPAddress _remote_ip;
  uint16_t _remote_port;
};

}  // namespace tmc
