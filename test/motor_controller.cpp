
#include <Arduino.h>

#include "motor_controller.h"

static motor_pin_map_t pin_map_l = {6, 4, 7, DUAL_DIRECTION_PIN};

MotorController m(&pin_map_l); // instantiate motor controller

static int i = 1;

void setup()
{
  Serial.begin(115200);

  Serial.println("motor driver test");

  uint16_t del = 500;

  m.init(); // test init
  Serial.print("init: ");
  Serial.print(digitalRead(pin_map_l.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map_l.direction_pin_reverse));
  Serial.println();

  delay(del);

  m.set_direction(MOTOR_DIRECTION_FORWARD); // test set direction
  Serial.print("direction forward: ");
  Serial.print(digitalRead(pin_map_l.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map_l.direction_pin_reverse));
  Serial.println();

  delay(del);

  m.set_direction(MOTOR_DIRECTION_REVERSE);
  Serial.print("direction reverse: ");
  Serial.print(digitalRead(pin_map_l.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map_l.direction_pin_reverse));
  Serial.println();

  delay(del);

  m.set_effort(MOTOR_EFFORT_MAX); // test set effort
  Serial.print("set effort max: ");
  Serial.print(digitalRead(pin_map_l.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map_l.direction_pin_reverse));
  Serial.println();

  delay(del);

  m.stop(); // test stop
  Serial.print("stop: ");
  Serial.print(digitalRead(pin_map_l.direction_pin));
  Serial.print(", ");
  Serial.print(digitalRead(pin_map_l.direction_pin_reverse));
  Serial.println();

  delay(del);
}

void loop()
{
  Serial.print("ramp forward:");
  for(int j = 0; j <= MOTOR_EFFORT_MAX / 2; j++)
  {
    m.set_vector_effort(i * j++); // test set vector effort

    delay(5);
  }

  Serial.println(m.get_vector_effort());

  delay(1000);

  m.stop();

  delay(1000);

  i *= -1;

  Serial.print("ramp reverse:");
  for(int j = 0; j <= MOTOR_EFFORT_MAX / 2; j++)
  {
    m.set_vector_effort(i * j++);
    delay(5);
  }

  Serial.println(m.get_vector_effort());

  m.stop();

  Serial.println("done test");

  // while(!Serial.available()) {} // wait for electrical test
  // while(Serial.available()) {Serial.read();}

  exit(0);
}
