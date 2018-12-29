#include <Arduino.h>

#include "motor_controller.h"

static motor_pin_map_t pin_map_l = {5, 3, 2, DUAL_DIRECTION_PIN};
static motor_pin_map_t pin_map_r = {6, 4, 7, DUAL_DIRECTION_PIN};

MotorController l(&pin_map_l); // instantiate motor controller
MotorController r(&pin_map_r);

void setup()
{
  Serial.begin(115200);

  Serial.println("double motor driver test");

  uint16_t del = 2000;

  l.init(); // init
  r.init();
  l.set_direction(MOTOR_DIRECTION_FORWARD);
  r.set_direction(MOTOR_DIRECTION_FORWARD);

  delay(del);

  Serial.println("test forward drive: "); // move forward
  l.set_effort(MOTOR_EFFORT_MAX/4);
  r.set_effort(MOTOR_EFFORT_MAX/4);
  Serial.print("left: ");
  Serial.print(digitalRead(pin_map_l.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map_l.direction_pin_reverse));
  Serial.print(" | right: ");
  Serial.print(digitalRead(pin_map_r.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map_r.direction_pin_reverse));
  Serial.println();

  delay(del);

  l.stop(); // stop
  r.stop();
}

void loop()
{
  // while(!Serial.available()) {} // wait for electrical test
  // while(Serial.available()) {Serial.read();}

  exit(0);
}
