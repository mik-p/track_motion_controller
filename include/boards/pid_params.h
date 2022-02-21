#pragma once

namespace tmc
{
#if defined(TMC_E407)
#define TMC_POS_PID_KP 0.3
#define TMC_POS_PID_KI 0.3
#define TMC_POS_PID_KD 0.1
#define TMC_VEL_PID_KP 0.3
#define TMC_VEL_PID_KI 0.1
#define TMC_VEL_PID_KD 0.05
#define TMC_PID_VEL_TO_EFF 3.1
#define TMC_PID_PULSE_TO_POS 0.0184
#else
#define TMC_POS_PID_KP 0.3
#define TMC_POS_PID_KI 0.3
#define TMC_POS_PID_KD 0.1
#define TMC_VEL_PID_KP 0.3
#define TMC_VEL_PID_KI 0.1
#define TMC_VEL_PID_KD 0.05
#define TMC_PID_VEL_TO_EFF 3.1
#define TMC_PID_PULSE_TO_POS 0.0184
#endif
}  // namespace tmc
