#pragma once

#include "boards/pin_defs.h"

#include "interface_controller.h"

namespace tmc
{
#if defined(TMC_E407)
// device mac address
byte TMC_MAC_ADDR_BYTE_ARR[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// make an interface controller
// SerialInterfaceController interface_c(&TMC_SERIAL_CONTROL, 115200);
UDPInterfaceController interface_c(0, TMC_MAC_ADDR_BYTE_ARR, IPAddress(172, 100, 22, 21));

// setup interfaces
void setup_interfaces()
{
  interface_c.init();
}

#else
// make an interface controller
SerialInterfaceController interface_c(&TMC_SERIAL_CONTROL, 9600);
#endif

}  // namespace tmc
