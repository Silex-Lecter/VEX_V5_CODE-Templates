#include "ui.hpp"
#include "parameters.hpp"
#include "robot-config.hpp"

void printMotorData() {
  Brain.Screen.printAt(10, 20, "BaseL1: t = %2.0f, installed = %1d",
                       Motor_BaseL1.temperature(vex::temperatureUnits::celsius),
                       Motor_BaseL1.installed());
  Brain.Screen.printAt(10, 40, "BaseL2: t = %2.0f, installed = %1d",
                       Motor_BaseL2.temperature(vex::temperatureUnits::celsius),
                       Motor_BaseL2.installed());
  Brain.Screen.printAt(10, 60, "BaseR1: t = %2.0f, installed = %1d",
                       Motor_BaseR1.temperature(vex::temperatureUnits::celsius),
                       Motor_BaseR1.installed());
  Brain.Screen.printAt(10, 80, "BaseR2: t = %2.0f, installed = %1d",
                       Motor_BaseR2.temperature(vex::temperatureUnits::celsius),
                       Motor_BaseR2.installed());
  Brain.Screen.printAt(
      10, 100, "Elevator1: t = %2.0f, installed = %1d",
      Motor_Elevator1.temperature(vex::temperatureUnits::celsius),
      Motor_Elevator1.installed());
  Brain.Screen.printAt(
      10, 120, "Elevator2: t = %2.0f, installed = %1d",
      Motor_Elevator2.temperature(vex::temperatureUnits::celsius),
      Motor_Elevator2.installed());
  Brain.Screen.printAt(10, 140, "Arm D1: t = %2.0f, installed = %1d",
                       Motor_Arm1.temperature(vex::temperatureUnits::celsius),
                       Motor_Arm1.installed());
}

void ui_task() {
  while (true) {
    printMotorData();
    Brain.Screen.printAt(10, 160, "IMU: t = %4.0f, installed = %1d",
                         IMU.rotation(vex::rotationUnits::deg),
                         IMU.installed());
    Brain.Screen.printAt(10, 180, "Limit: t = %1d",
                         Elevator_Lower_limit.pressing());
    Brain.Screen.printAt(10, 200, "Pose: x = %4.0f, y = %4.0f, theta = %4.1f",
                         Odom.getPose().x, Odom.getPose().y,
                         rad2deg(Odom.getPose().theta));
    vex::this_thread::sleep_for(50);
  }
}