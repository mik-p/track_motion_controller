
#include <Arduino.h>

#include "pid_controller.h"

static pid_parameters_t pin_map_l = {0.5, 0.1, 0.2, 0, 0, 0, 0};

PIDController p(&pin_map_l); // instantiate pid controller

void setup()
{
  Serial.begin(115200);

  Serial.println("pid controller test");

  uint16_t del = 2000;

  delay(del);

  Serial.println("setters and getters:");
  p.setpoint(100);
  Serial.println(p.setpoint());

  p.measurement(0);
  Serial.println(p.measurement());

  p.add_measurement(1);
  Serial.println(p.measurement());

  delay(del);

  Serial.println("pid factory:");
}

void loop()
{
  if(p.measurement() < p.setpoint())
  {
    p.add_measurement(1);
  }
  else
  {
    p.add_measurement(-1);
  }

  Serial.println(p.pid_factory(0.1));

  while(!Serial.available()) {} // wait for electrical test
  while(Serial.available()) {Serial.read();}

  // exit(0);
}
