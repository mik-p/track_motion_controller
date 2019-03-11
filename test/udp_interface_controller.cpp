
#include "interface_controller.h"

static udp_interface_parameters_t udp_params = {
  10,
  {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED},
  IPAddress(192, 168, 1, 177),
  INTERFACE_DEFAULT_CIP_UDP_TCP_PORT,
};

static UDPInterfaceController udp_int(&udp_params);

static uint16_t del = 100;

void setup()
{
  Serial.begin(115200);
  udp_int.begin();

  Serial.println("UDP interface test");

  delay(del);
}

void loop()
{
  // int packetSize = udp_int._udp.parsePacket();
  udp_int.printf("UDP interface test\n");

  // if(udp_int.send_and_receive())
  // {
    // udp_int.printf("%d\n", ser_int.get_control_msg().msg_status);
    // udp_int.set_feedback_msg({1});
  // }

  delay(del);

  // exit(0);
}
