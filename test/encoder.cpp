
#include <Arduino.h>

#include "encoder.h"

using namespace tmc;

Encoder::encoder_pin_map_t pin_map = { 2, 4 };

Encoder e = Encoder(pin_map);

double pos = 0;
double vel = 0;

double ratio = 1.0 / 30.0;

unsigned long t = 0;

void ENC_ISR()
{
  e.tick();
}

void setup()
{
  Serial.begin(115200);

  Serial.println("encoder test");

  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);

  e.init();  // test init

  e.set_tick_interrupt(INT0, ENC_ISR);  // test set interrupt

  uint16_t del = 2000;

  delay(del);
}

void loop()  // test get pos and vel
{
  t = millis();

  uint16_t del = 100;
  delay(del);

  unsigned long dt = millis() - t;

  double disp = e.get_displacement(pos, ratio);

  pos += disp;

  vel = e.get_velocity(disp, (double)dt / 1000);

  Serial.print(pos);
  Serial.print(", ");
  Serial.println(vel);
  // exit(0);
}
