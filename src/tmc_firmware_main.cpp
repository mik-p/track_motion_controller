
#include <Arduino.h>

#include "platform/platform.h"

void setup()
{
  tmc::tmc_setup(__FILE__);
}

void loop()
{
  tmc::tmc_loop();
}
