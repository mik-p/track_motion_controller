
#include "interface_controller.h"

static serial_interface_parameters_t serial_params = {
  INTERFACE_DEFAULT_SERIAL_BAUD,
  &Serial
};

static SerialInterfaceController ser_int(&serial_params);

static uint16_t del = 100;

void setup()
{
  ser_int.begin();

  ser_int.printf("Serial interface test\n");

  delay(del);
}

void loop()
{
  if(ser_int.send_and_receive())
  {
    ser_int.printf("%d\n", ser_int.get_control_msg().msg_status);
    ser_int.set_feedback_msg({1});
  }

  delay(del);

  // exit(0);
}
