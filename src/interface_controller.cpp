
#include "interface_controller.h"


int InterfaceController::send_and_receive()
{
  // if feedback is new send through interface
  if(_feedback_msg.msg_status == INTERFACE_MSG_STATUS_PENDING)
  {
    char buf[INTERFACE_MAX_BUFFER_SIZE];

    uint16_t len = serialise(buf, INTERFACE_MAX_BUFFER_SIZE, &_feedback_msg);

    _send_feedback_msg(buf, len); // send feedback to controller

    _feedback_msg.msg_status = INTERFACE_MSG_STATUS_DONE;
  }

  return _receive_control_msg(); // receive message from interface
}

// TODO actually write serialisers
uint16_t InterfaceController::serialise(char * buf, uint16_t len, interface_msg_t * msg)
{
  uint16_t result_len = 0;

  buf[0] = 'h'; buf[1] = 'i';

  result_len = 2;

  return result_len;
}

interface_msg_t InterfaceController::deserialise(char * buf, uint16_t len)
{
  interface_msg_t msg = {(uint8_t)buf[0]};
  return msg;
}

void InterfaceController::printf(const char *fmt, ...)
{
  char buf[INTERFACE_MAX_BUFFER_SIZE];

  // fill buffer from format and var list
  va_list va;
  va_start(va, fmt);
  vsprintf(buf, fmt, va);
  va_end(va);

  _print(buf); // output to stream
}

SerialInterfaceController::SerialInterfaceController(serial_interface_parameters_t * params) :
_serial_params(params),
_buffer_index(0)
{}

int SerialInterfaceController::_receive_control_msg()
{
  while(_serial_params->serial_port->available())
  {
    // TODO add timeout for listen
    // TODO add length flag for listen
    _buffer[_buffer_index++] = (char)_serial_params->serial_port->read();

    if(_buffer[_buffer_index-1] == '\n') // end of message control character
    {
      _control_msg = deserialise(_buffer, _buffer_index-1); // set control message

      _buffer_index = 0; // reset buffer index

      return 1; // return 1 for new message
    }
  }

  return 0;
}

void SerialInterfaceController::_send_feedback_msg(char * buf, uint16_t len)
{
  _serial_params->serial_port->write(buf, len); // write message
  _serial_params->serial_port->write('\n'); // add control character
}

UDPInterfaceController::UDPInterfaceController(udp_interface_parameters_t * params) :
_udp_params(params)
{}

void UDPInterfaceController::begin()
{
  Ethernet.init(_udp_params->spi_cs);
  Ethernet.begin(_udp_params->mac, _udp_params->ip_address);
  _udp.begin(_udp_params->port);
}
