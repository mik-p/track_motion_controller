
#include <Arduino.h>

#include "encoder.h"
#include "pid_controller.h"

// instantiate encoder and pid controller
static encoder_pin_map_t pin_map = {2, 4};

static Encoder e(&pin_map);

static pid_parameters_t pin_map_l = {0.5, 0.1, 0.2, 0, 0, 0, 0, 100, -100};

static PIDController p(&pin_map_l);

double setpoint = 10;

double ratio = 1.0 / 30.0;

unsigned long t = 0;

void ENC_ISR()
{
  e.tick();
}

void setup()
{
  Serial.begin(115200);

  Serial.println("pid controller with encoder test");

  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);

  e.init();

  e.set_tick_interrupt(INT0, ENC_ISR);

  Serial.print("set control point: ");
  p.setpoint(setpoint);
  Serial.println(p.setpoint());

  p.measurement(e.get_displacement(0, ratio));

  uint16_t del = 2000;

  delay(del);

  Serial.println("pid factory with feedback:");
}

void loop()
{
  t = millis(); // start sample

  uint16_t del = 100;
  delay(del);

  unsigned long dt = millis() - t;

  p.add_measurement(e.get_displacement(p.measurement(), ratio)); // update pid control measurement

  Serial.print(p.measurement());
  Serial.print(", ");
  Serial.println(p.improved_pid_factory((double)dt / 1000)); // get control signal

  // while(!Serial.available()) {} // wait for verification test
  // while(Serial.available()) {Serial.read();}

  // exit(0);
}
