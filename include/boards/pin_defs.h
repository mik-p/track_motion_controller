#pragma once

#include <Arduino.h>

#if defined(TMC_E407)
// LED
#if defined(LED_BUILTIN)
#undef LED_BUILTIN
#define LED_BUILTIN PC13
#endif
// motor pins
#define L_MOTOR_PIN_0 PD3
#define L_MOTOR_PIN_1 PD4
#define R_MOTOR_PIN_0 PD3
#define R_MOTOR_PIN_1 PD4
// encoder pins
#define L_ENC_PIN_0 PD5
#define L_ENC_PIN_1 PD6
#define R_ENC_PIN_0 PD7
#define R_ENC_PIN_1 PD8
// interrupt pins
#define L_ENC_PIN_INT L_ENC_PIN_0
#define R_ENC_PIN_INT R_ENC_PIN_0
// serial
#define TMC_SERIAL_CONFIG Serial
// #define TMC_SERIAL_CONTROL Serial3
#define TMC_SERIAL_CONTROL Serial6
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
// interrupt pins
#define L_ENC_PIN_INT INT0
#define R_ENC_PIN_INT INT2
// serial
#define TMC_SERIAL_CONFIG Serial
#define TMC_SERIAL_CONTROL Serial
#endif
