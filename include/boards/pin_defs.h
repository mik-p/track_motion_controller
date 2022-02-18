#pragma once

#if TMC_E407
// LED
#define LED_BUILTIN PC13
// motor pins
#define L_MOTOR_PIN_0 5
#define L_MOTOR_PIN_1 3
#define L_MOTOR_PIN_2 2
#define R_MOTOR_PIN_0 6
#define R_MOTOR_PIN_1 4
#define R_MOTOR_PIN_2 7
// encoder pins
#define L_ENC_PIN_0 21
#define L_ENC_PIN_1 19
#define R_ENC_PIN_0 19
#define R_ENC_PIN_1 19
// serial
#define TMC_SERIAL_CONFIG Serial
#define TMC_SERIAL_CONTROL Serial1
#else
// motor pins
#define L_MOTOR_PIN_0 5
#define L_MOTOR_PIN_1 3
#define L_MOTOR_PIN_2 2
#define R_MOTOR_PIN_0 6
#define R_MOTOR_PIN_1 4
#define R_MOTOR_PIN_2 7
// encoder pins
#define L_ENC_PIN_0 21
#define L_ENC_PIN_1 19
#define R_ENC_PIN_0 19
#define R_ENC_PIN_1 19
// serial
#define TMC_SERIAL_CONFIG Serial
#define TMC_SERIAL_CONTROL TMC_SERIAL_CONFIG
#endif
