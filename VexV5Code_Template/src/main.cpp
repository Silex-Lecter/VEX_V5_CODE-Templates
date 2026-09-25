#include "api.h"
#include "autonomous.hpp"
#include "parameters.hpp"
#include "robot-config.hpp"
#include "ui.hpp"
#include "usercontrol.hpp"
#include "module.hpp"

vex::competition Competition;

void odom_task() {
  while (true) {
    Odom.update();
    vex::this_thread::sleep_for(5);
  }
}

void ele_task() {
  Ele_D1.setSlewRate(0);
  while (true) {
    Ele_D1.update();
    vex::this_thread::sleep_for(10);
  }
}

int main() {
  printf("//------------------2700 Program Start------------------//\n");
  initRobot();
  vex::thread ui_thread(ui_task);
  vex::thread odom_thread(odom_task);
  vex::thread arm_thread(arm_task);
  vex::thread ele_thread(ele_task);
  vex::thread toggle_thread(toggle_task);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    vex::this_thread::sleep_for(100);
  }
}
