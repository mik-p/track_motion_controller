
#include "motion_controller.h"


SkidSteerController::SkidSteerController() :
{}

void printMotorInfo() {
  if((millis() - lastTime_print) >= 500) {
    lastTime_print = millis();
    Serial.print("Setpoint: ");    Serial.println(speed_req);
    Serial.print("Speed RPM: ");    Serial.println(speed_actual);
  }
}
