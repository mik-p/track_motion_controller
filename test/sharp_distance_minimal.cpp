
#include <Arduino.h>

#include "motor_controller.h"

#define SHARP_PIN A0
#define LIMITER 2

static motor_pin_map_t pin_map_l = {11, 13, 0, SINGLE_DIRECTION_PIN};
static motor_pin_map_t pin_map_r = {10, 12, 0, SINGLE_DIRECTION_PIN};

MotorController l(&pin_map_l); // instantiate motor controller
MotorController r(&pin_map_r);

uint16_t get_gp2d12(uint16_t value)
{
	if (value < 10) value = 10;
	return ((67870.0 / (value - 3.0)) - 40.0);
}

void forward()
{
  l.set_direction(MOTOR_DIRECTION_FORWARD);
  r.set_direction(MOTOR_DIRECTION_FORWARD);
  l.set_effort(MOTOR_EFFORT_MAX/LIMITER);
  r.set_effort(MOTOR_EFFORT_MAX/LIMITER);
}

void reverse()
{
  l.set_direction(MOTOR_DIRECTION_REVERSE);
  r.set_direction(MOTOR_DIRECTION_REVERSE);
  l.set_effort(MOTOR_EFFORT_MAX/LIMITER);
  r.set_effort(MOTOR_EFFORT_MAX/LIMITER);
}

void stop()
{
  l.stop();
  r.stop();
}

void turn_right()
{
  l.set_direction(MOTOR_DIRECTION_FORWARD);
  r.set_direction(MOTOR_DIRECTION_REVERSE);
  l.set_effort(MOTOR_EFFORT_MAX/LIMITER);
  r.set_effort(MOTOR_EFFORT_MAX/LIMITER);
}

void turn_left()
{
  l.set_direction(MOTOR_DIRECTION_REVERSE);
  r.set_direction(MOTOR_DIRECTION_FORWARD);
  l.set_effort(MOTOR_EFFORT_MAX/LIMITER);
  r.set_effort(MOTOR_EFFORT_MAX/LIMITER);
}

void setup()
{

  // Serial.println("double motor driver test");

  uint16_t del = 1000;

  l.init(); // init
  r.init();
  stop();
  // r.set_direction(MOTOR_DIRECTION_FORWARD);
  // l.set_direction(MOTOR_DIRECTION_FORWARD);

  Serial.begin(115200);

  pinMode(SHARP_PIN, INPUT);

  delay(del);

  // Serial.println("test forward drive: "); // move forward
  // l.set_effort(MOTOR_EFFORT_MAX/4);
  // r.set_effort(MOTOR_EFFORT_MAX/4);
  // Serial.print("left: ");
  // Serial.print(digitalRead(pin_map_l.direction_pin));
  // Serial.print(", ");
  // Serial.print(digitalRead(pin_map_l.direction_pin_reverse));
  // Serial.print(" | right: ");
  // Serial.print(digitalRead(pin_map_r.direction_pin));
  // Serial.print(", ");
  // Serial.print(digitalRead(pin_map_r.direction_pin_reverse));
  // Serial.println();

  // delay(del);
}

void loop()
{
  uint8_t del = 500;

  uint16_t value = analogRead(SHARP_PIN);
	uint16_t range = get_gp2d12(value);

  // Serial.println(range);

  if(range < 75) // 60 mm
  {
    stop();
    delay(del);
    reverse();
    delay(del * 6);
    turn_left();
    delay(del * 3);
  }
  else
  {
    forward();
  }

  // while(!Serial.available()) {} // wait for electrical test
  // while(Serial.available()) {Serial.read();}

  // exit(0);
}
