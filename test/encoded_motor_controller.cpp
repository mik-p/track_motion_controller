
#include <Arduino.h>

#include "motor_controller.h"

motor_pin_map_t r_pins = {6, 4, 7, DUAL_DIRECTION_PIN}; // stop second motor
static MotorController r_motor(&r_pins);

// instantiate config and motor class
motor_pin_map_t m_pin_map = {5, 3, 2, DUAL_DIRECTION_PIN};
encoder_pin_map_t e_pin_map = {21, 19};
double vel_to_effort = 3.1;
double pulse_to_pos = 0.0184;
unsigned long dt = 100; // ms
pid_parameters_t pos_pid = {0.5, 0.1, 0.1, 0, 0, 0, 0, 100, -100};
pid_parameters_t vel_pid = {0.6, 0.01, 0.2, 0, 0, 0, 0, 100, -100};

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

double setpoint = 50;

uint8_t start_effort = 0;

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

  Serial.print("effort calibration: ");
  double effort_to_vel = 0;
  double sum = 0;
  int n = 0;
  for(int i=1; i<=10; i++)
  {
    uint8_t effort = (i * MOTOR_EFFORT_MAX) / 10; // choose some test speeds

    effort_to_vel = em.tune_effort_scalar(effort);
    if(effort_to_vel)
    {
      sum += effort_to_vel;
      n++;
    }
    else
    {
      start_effort = ((i + 1) * MOTOR_EFFORT_MAX) / 10;
    }

    delay(del);

    Serial.print(effort_to_vel);
    Serial.print(", ");
  }
  Serial.println();
  Serial.print("effort to velocity model: ");
  if(sum != 0)
  {
    params.vel_to_effort = n / sum; // flipped during calculation
    Serial.println(params.vel_to_effort);
  }
  else
  {
    Serial.println("failed");
  }

  delay(del);

  Serial.print("set control point: ");
  Serial.println(setpoint);
  // em.set_position(setpoint);
  em.set_velocity(setpoint);

  Serial.print("current pos / vel: ");
  Serial.print(em.get_position());
  Serial.print(" / ");
  Serial.println(em.get_velocity());

  Serial.println("begin motion");
  em.set_vector_effort(start_effort);
  delay(del);
}

void loop()
{
  em.update(); // run control loop

  // view result
  Serial.print(em.get_position());
  Serial.print(", ");
  Serial.print(em.get_velocity());
  Serial.print(", ");
  Serial.println(em.get_vector_effort());

  delay(100);

  // while(!Serial.available()) {} // wait for verification test
  while(Serial.available())
  {
    // Serial.read();
    em.stop();
    exit(0);
  }

  // exit(0);
}
