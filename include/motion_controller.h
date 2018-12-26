
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "motor_controller.h"


class MotionController
{
public:
protected:
};

class SkidSteerController : public MotionController
{
public:
  SkidSteerController();

private:
  EncodedMotorController _left_motor, _right_motor;
};

#endif
