#include "autonomous.hpp"

#include "lib/chassis/differential-chassis.hpp"
#include "robot-config.hpp"
#include "usercontrol.hpp"

static auto auton_timer = vex::timer();
static int auton_strategy = 0;

#define AUTON_ROUTE_NUMBER 6

void auton_init(void) {
  // init
  auton_timer.reset();
  enableUsercontrol();
}

void auton_pre_usercontrol(void) {
  Brain.Screen.setCursor(12, 1);
  Brain.Screen.print("AutonTimer: %2.2fsec", auton_timer.time() / 1000.0);
  printf("AutonTimer: %2.2fsec\n", auton_timer.time() / 1000.0);
}
int getAutonTimer() { return auton_timer.time(); }

void changeAutonStrategy(void) {
  auton_strategy++;
  auton_strategy = auton_strategy % AUTON_ROUTE_NUMBER;
  switch (auton_strategy) {
    case 0:
      Controller.Screen.setCursor(5, 1);
      Controller.Screen.print("%12s", "left-13");
      break;
    case 1:
      Controller.Screen.setCursor(5, 1);
      Controller.Screen.print("%12s", "right-13");
      break;
    case 2:
      Controller.Screen.setCursor(5, 1);
      Controller.Screen.print("%12s", "left-23");
      break;
    case 3:
      Controller.Screen.setCursor(5, 1);
      Controller.Screen.print("%12s", "right-23");
      break;
    case 4:
      Controller.Screen.setCursor(5, 1);
      Controller.Screen.print("%12s", "left-22");
      break;
    case 5:
      Controller.Screen.setCursor(5, 1);
      Controller.Screen.print("%12s", "right-22");
      break;
    case 6:
      Controller.Screen.setCursor(5, 1);
      Controller.Screen.print("%12s", "none");
      break;
  }
}

void autonomous(void) {
  switch (auton_strategy) {
    case 0:
      auton_left_13();
      break;
    case 1:
      auton_right_13();
      break;
    case 2:
      auton_left_23();
      break;
    case 3:
      auton_right_23();
      break;
    case 4:
      auton_left_22();
      break;
    case 5:
      auton_right_22();
      break;
    case 6:
      break;
  }
}
