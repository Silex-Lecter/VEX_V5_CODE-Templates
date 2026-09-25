#pragma once
#include "v5.h"
#include "v5_vcs.h"
#include <vector>

class MotorGroup {
public:
  template <typename... Args> MotorGroup(Args&... ms) {
    static_assert(sizeof...(Args) >= 1 && sizeof...(Args) <= 15,
                  "MotorGroup supports 1~15 motors");
    motors = {&ms...};
  }
  void spin(vex::directionType dir, double speed, vex::velocityUnits units) {
    for (auto m : motors)
      m->spin(dir, speed, units);
  }
  void spin(vex::directionType dir, double volt, vex::voltageUnits units) {
    for (auto m : motors)
      m->spin(dir, volt, units);
  }
  void spin(double volt) {
    for (auto m : motors)
      m->spin(vex::forward, volt * 127, vex::voltageUnits::mV);
  }
  void stop(vex::brakeType mode) {
    for (auto m : motors)
      m->stop(mode);
  }
  double position(vex::rotationUnits units) {
    double sum = 0;
    for (auto m : motors)
      sum += m->position(units);
    return sum / motors.size();
  }
  void resetPosition() {
    for (auto m : motors)
      m->resetPosition();
  }
  double velocity(vex::velocityUnits units) {
    double sum = 0;
    for (auto m : motors)
      sum += m->velocity(units);
    return sum / motors.size();
  }
  double temperature() {
    double max_temp = motors[0]->temperature();
    for (auto m : motors) {
      if (m->temperature() > max_temp) max_temp = m->temperature();
    }
    return max_temp;
  }
  vex::gearSetting getMotorCartridge() {
    return motors[0]->getMotorCartridge();
  }

private:
  std::vector<vex::motor*> motors;
};
