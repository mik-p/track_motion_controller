
#include <Arduino.h>

#include "motor_controller.h"

motor_pin_map_t r_pins = {6, 4, 7, DUAL_DIRECTION_PIN}; // stop second motor
static MotorController r_motor(&r_pins);

// instantiate config and motor class
motor_pin_map_t m_pin_map = {5, 3, 2, DUAL_DIRECTION_PIN};
encoder_pin_map_t e_pin_map = {21, 19};
double vel_to_effort = 3.1;
double pulse_to_pos = 0.0184;
unsigned long dt = 50; // ms
pid_parameters_t pos_pid = {0.3, 0.3, 0.1, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort)};
pid_parameters_t vel_pid = {0.3, 0.1, 0.05, 0, 0, 0, 0, -(255 / vel_to_effort), (255 / vel_to_effort)};

static encoded_motor_parameters_t params = {
  m_pin_map,
  e_pin_map,
  vel_to_effort,
  pulse_to_pos,
  dt,
  pos_pid,
  vel_pid
};

static EncodedMotorController em(&params);

// double pos_setpoint = 500;
double setpoint = 0;

uint8_t start_effort = 60;

void ENC_ISR()
{
  em.encoder_tick();
}

void setup()
{
  Serial.begin(115200);

  Serial.println("encoded motor controller test");

  r_motor.init();

  // pinMode(20, OUTPUT); // for testing with incremental encoder
  // digitalWrite(20, LOW);

  em.init();

  em.set_encoder_interrupt(INT2, ENC_ISR);

  uint16_t del = 2000;

  delay(del);

  Serial.print("set control point: ");
  Serial.println(setpoint);
  // em.set_position(pos_setpoint);
  // em.set_velocity(vel_setpoint);

  Serial.print("current pos / vel: ");
  Serial.print(em.get_position());
  Serial.print(" / ");
  Serial.println(em.get_velocity());

  Serial.println("begin motion");
}

void loop()
{
  char inchar = 'f';
  while(inchar != 's')
  {
    em.update(); // run control loop

    // view result
    Serial.print(em.get_position());
    Serial.print(", ");
    Serial.print(em.get_velocity());
    Serial.print(", ");
    Serial.println(em.get_vector_effort());

    delay(dt);

    inchar = Serial.read();
    if(inchar == 'i')
    {
      setpoint = 1;
      em.set_position(setpoint);
      // em.set_velocity(setpoint);
      Serial.println(setpoint);
    }
    else if(inchar == 'k')
    {
      setpoint = -1;
      em.set_position(setpoint);
      // em.set_velocity(setpoint);
      Serial.println(setpoint);
    }
  }

  inchar = 'f';
  // while(!Serial.available()) {} // wait for verification test
  while(inchar != 's')
  {
    em.stop();

    inchar = Serial.read();
    if(inchar == 'i')
    {
      setpoint++;
      em.set_position(setpoint);
      // em.set_velocity(setpoint);
      Serial.println(setpoint);
    }
    else if(inchar == 'k')
    {
      setpoint--;
      em.set_position(setpoint);
      // em.set_velocity(setpoint);
      Serial.println(setpoint);
    }
  }

  // exit(0);
}
