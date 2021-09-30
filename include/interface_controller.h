#pragma once

#include <HardwareSerial.h>
// #include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

namespace tmc
{
#ifndef BYTE_ORDER_UTIL_MACROS
#define BYTE_ORDER_UTIL_MACROS

#define htons(x) ((((x) << 8) & 0xFF00) | (((x) >> 8) & 0xFF))
#define ntohs(x) htons(x)

#define htonl(x)                                                                                                       \
  (((x) << 24 & 0xFF000000UL) | ((x) << 8 & 0x00FF0000UL) | ((x) >> 8 & 0x0000FF00UL) | ((x) >> 24 & 0x000000FFUL))
#define ntohl(x) htonl(x)

#endif

// defining a BIG_ENDIAN representation of our data (network byte order)
#define INTERFACE_HEADER_SIZE (sizeof(uint8_t) + (3 * sizeof(uint16_t)))
#define INTERFACE_MAX_DATA_SIZE 8
#define INTERFACE_MAX_BUFFER_SIZE (INTERFACE_HEADER_SIZE + (INTERFACE_MAX_DATA_SIZE * sizeof(float)))

typedef struct
{
  uint8_t status;      // 8 bits to use for status flags
  uint16_t loop_time;  // in milliseconds
  uint16_t sequence;   // incrementing sequence number
  uint16_t length;     // data buffer length
  float* data;         // the data
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

  interface_msg_t* get_control_msg_ptr()
  {
    return &_control_msg;
  }

  void set_feedback_msg_header(const uint16_t& loop_time)
  {
    _feedback_msg.status = 0;
    _feedback_msg.loop_time = loop_time;
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

protected:
  InterfaceController()
  {
    // init feedback msg
    _feedback_msg.status = 0;
    _feedback_msg.loop_time = 0;
    _feedback_msg.length = 0;
    _feedback_msg.sequence = 0;

    // setup the msg buffers
    _feedback_msg.data = _feedback_data;
    _control_msg.data = _control_data;

    memset(_feedback_msg.data, 0, INTERFACE_MAX_DATA_SIZE);
    memset(_control_msg.data, 0, INTERFACE_MAX_DATA_SIZE);
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

    // sequence
    buf[result_len++] = _feedback_msg.sequence >> 8;      // high byte
    buf[result_len++] = _feedback_msg.sequence & 0x00FF;  // low byte

    // length
    buf[result_len++] = _feedback_msg.length >> 8;      // high byte
    buf[result_len++] = _feedback_msg.length & 0x00FF;  // low byte

    // data in floats
    for (uint8_t i = 0; i < _feedback_msg.length; ++i)
    {
      memcpy((void*)&buf[result_len], (void*)htonl(*(long*)&_feedback_msg.data[i]), sizeof(long));
      result_len += sizeof(float);
    }

    // return the buffer length
    return result_len;
  }

  interface_msg_t* _deserialise(char* buf, const uint16_t& len)
  {
    uint16_t result_len = 0;

    if (len > INTERFACE_HEADER_SIZE)
    {
      // first byte is status
      _control_msg.status = buf[result_len++];

      // second two bytes is loop time
      _control_msg.loop_time = ((uint16_t)buf[result_len] << 8) | buf[result_len + 1];
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
    }

    // the rest of data is sets of sizeof(float) length long
    if (len - result_len == _control_msg.length * sizeof(float))
    {
      for (uint8_t i = 0; i < _control_msg.length; ++i)
      {
        memcpy((void*)&_control_msg.data[i], (void*)ntohl(*(long*)&buf[result_len]), sizeof(float));
        result_len += sizeof(long);
      }
    }

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
    interface_msg_t* msg_ptr = _deserialise(buf, len);

    return msg_ptr->length;
  }

  void _send_feedback_msg()
  {
    // create buffer
    uint16_t len = INTERFACE_MAX_BUFFER_SIZE;
    char buf[len];

    // serialise the msg into the buffer
    len = _serialise(buf, len, _feedback_msg);

    // send the buffer
    _send_buffer(buf, len);
  }

  // interface specific read and write calls
  virtual void _send_buffer(const char* buf, const uint16_t len) = 0;
  virtual const uint16_t _recv_buffer(char* buf, const uint16_t len) = 0;

protected:
  // the data storage for the interface
  interface_msg_t _control_msg, _feedback_msg;
  float _feedback_data[INTERFACE_MAX_DATA_SIZE];
  float _control_data[INTERFACE_MAX_DATA_SIZE];
};

class SerialInterfaceController : public InterfaceController
{
  // define read and write for serial interface

public:
  SerialInterfaceController(HardwareSerial* ser, const unsigned int baud)
  {
    _port = ser;
    _port->begin(baud);
  }

protected:
  virtual void _send_buffer(const char* buf, const uint16_t len)
  {
    _port->write(buf, len);
    _port->println();  // print a '\n' as a control character
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
};

class UDPInterfaceController : public InterfaceController
{
  // define read and write for udp interface

public:
  UDPInterfaceController(uint8_t spi_cs, uint8_t* mac, IPAddress ip_address)
  {
    Ethernet.init(spi_cs);
    Ethernet.begin(mac, ip_address);
    _udp.begin(2021);
  }

protected:
  virtual void _send_buffer(const char* buf, const uint16_t len)
  {
    int check = _udp.beginPacket(_udp.remoteIP(), _udp.remotePort());
    _udp.write(buf, len);
    _udp.endPacket();
  }

  virtual const uint16_t _recv_buffer(char* buf, const uint16_t len)
  {
  }

private:
  EthernetUDP _udp;
};

}  // namespace tmc
