#pragma once

namespace tmc
{
#if defined(TMC_E407)
// position pid variables
#define TMC_POS_PID_KP 14.0
#define TMC_POS_PID_KI 0.0
#define TMC_POS_PID_KD 0.0
// velocity pid variables
#define TMC_VEL_PID_KP 1.0
#define TMC_VEL_PID_KI 7.0
#define TMC_VEL_PID_KD 0.0001
// encoder pulses per revolution
#define TMC_ENC_PPR 48  // 48 ppr
// motor gear ratio
#define TMC_MOT_GEAR_RATIO 37.8  // 38:1 - 40:1 supposed to be
// pulses per revolution at output shaft
#define TMC_WHEEL_PPR 1814.4  // mot_gear_ratio * enc_ppr
// convert from rad velocity and nominal effort scalar
// #define TMC_PID_MAX_VEL 0.7 * 2 * M_PI  // max velocity revs/s * 2 * pi (r/s)
#define TMC_PID_MAX_VEL 4.39822971  // max velocity (r/s)
#define TMC_PID_VEL_TO_EFF 56.841051169  // max_eff / max_vel
// radial increment per pulse (arc length)
#define TMC_PID_PULSE_TO_POS 0.000551146  // (wheel_revs / (mot_revs * enc_ppr))
// linear position per pulse (includes wheel size)
#define TMC_PID_PULSE_TO_POS_LIN 0.0005522487  // (2 * pi * wheel_rad / (mot_gear_ratio * enc_ppr)) ??
#else
#define TMC_POS_PID_KP 0.3
#define TMC_POS_PID_KI 0.3
#define TMC_POS_PID_KD 0.1
#define TMC_VEL_PID_KP 0.3
#define TMC_VEL_PID_KI 0.1
#define TMC_VEL_PID_KD 0.05
#define TMC_ENC_PPR 20         // 200 ppr
#define TMC_MOT_GEAR_RATIO 15  // 35:1
#define TMC_PID_VEL_TO_EFF 3.1
#define TMC_PID_PULSE_TO_POS 0.0184
#endif
}  // namespace tmc
