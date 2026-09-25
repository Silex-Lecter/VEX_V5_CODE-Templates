#include "usercontrol.hpp"
#include "api.h"
#include "autonomous.hpp"
#include "module.hpp"
#include "parameters.hpp"
#include "robot-config.hpp"

namespace {
bool usercontrol_enable = true;
}

void enableUsercontrol() { usercontrol_enable = true; }

void usercontrol(void) {
  float elevator_output = 0;
  int arm_level = 0;
  bool is_auton_elevator_enabled = false;
  ArmEvent last_arm_event;

  vex::timer toggle_timer;
  while (1) {
    Con.update();
    /*-----------calibration mode----------------*/
    if (Con.UP && !Con.last_UP) {
      calibrate();
      usercontrol_enable = false;
    }
    /*-----------tuning mode----------------*/
    if (Con.DOWN) autonomous();
    if (Con.B && !Con.last_B) {
      printf("Elevator position: %f\n", Ele_D1.deg());
      printf("Arm D2 position: %f\n", Arm_D2.deg());
      printf("color: %d, hue: %f, near: %d\n", Toggle_Sensor1.classifyColor(),
             Color_Sensor1.hue(), Color_Sensor1.isNearObject());
      printf("color: %d, hue: %f, near: %d\n", Toggle_Sensor2.classifyColor(),
             Color_Sensor2.hue(), Color_Sensor2.isNearObject());
    }
    /*-------------Autonomous Strategy Control-------------*/
    if (Con.LEFT && !Con.last_LEFT) changeAutonStrategy();
    // Configure and invoke PidTuner here when interactive tuning is needed.
    if (Con.RIGHT && !Con.last_RIGHT) changeRobotColor();
    // if (Con.RIGHT && !Con.last_RIGHT) tuneChassisAngular();
    if (!usercontrol_enable) continue;

    /*-------------Base Movement Control-------------*/
    Con.A3 = getControllerCurveOutput(
        Con.A3, {.curve_exponent = 1.00, .deadzone = 2, .min_output = 3});
    Con.A1 = getControllerCurveOutput(
        Con.A1, {.curve_exponent = 1.00, .deadzone = 2, .min_output = 3});
    Chassis.move(Con.A3, -Con.A1);

    /*-------------Elevator D1 Control-------------*/
    // elevator output control
    if (Con.A2 > 60) {
      elevator_output = 100;
      is_auton_elevator_enabled = false;
    } else if (Con.A2 < -60) {
      elevator_output = -70;
      is_auton_elevator_enabled = false;
    } else {
      if (Piston_Claw.value() && getArmLevel() == ArmEvent::FRONT_BUTTON &&
          Ele_D1.deg() < 20) {
        elevator_output = 60;
      } else if (!Piston_Claw.value() && getArmLevel() == ArmEvent::FRONT_BUTTON) {
        elevator_output = -70;
      } else if (Piston_Claw.value() && getArmLevel() == ArmEvent::REAR_BUTTON &&
                 Ele_D1.deg() < 220 && is_auton_elevator_enabled) {
        elevator_output = 80;
      } else if (Piston_Claw.value() && getArmLevel() == ArmEvent::REAR_TOP &&
                 Ele_D1.deg() < 140 && is_auton_elevator_enabled) {
        elevator_output = 80;
      } else {
        elevator_output = 0;
      }
    }

    // elevator upper limit and lower limit protection
    if (Elevator_Lower_limit.pressing() && elevator_output <= 0) {
      elevator_output = -1;
    } else if (Ele_D1.deg() > 600 && elevator_output > 0) {
      elevator_output = 0;
    }

    // elevator output
    Ele_D1.runAtPower(elevator_output);

    // elevator lower limit reset
    if (Elevator_Lower_limit.pressing()) {
      Ele_D1.reset();
    }

    /*-------------Arm D2 Control-------------*/
    if (Con.L1 && !Con.last_L1 && arm_level < 3 && Piston_Claw.value()) {
      arm_level++;
    } else if (Con.L2 && !Con.last_L2) {
      if (arm_level > 0 && Piston_Claw.value()) {
        arm_level--;
      } else {
        arm_level = 0;
      }
    }

    if (Con.L1 || Con.L2) {
      switch (arm_level) {
        case 0:
          setArmLevel(ArmEvent::FRONT_BUTTON);
          break;
        case 1:
          setArmLevel(ArmEvent::FRONT_TOP);
          break;
        case 2:
          setArmLevel(ArmEvent::REAR_TOP);
          break;
        case 3:
          setArmLevel(ArmEvent::REAR_BUTTON);
          break;
      }
    }
    if (Con.Y) {
      if (getArmLevel() != ArmEvent::RESET) last_arm_event = getArmLevel();
      setArmLevel(ArmEvent::RESET);
    }
    if (!Con.Y && Con.last_Y) {
      setArmLevel(last_arm_event);
    }
    
    if (Con.A2 > 80 && Ele_D1.deg() > 270 && arm_level == 1 && Piston_Claw.value()) {
      setArmLevel(ArmEvent::TOP);
    }
    if(Con.A2 <= 80 && Con.last_A2 > 80 && getArmLevel() == ArmEvent::TOP && Piston_Claw.value()){
      setArmLevel(ArmEvent::FRONT_TOP);
    }

    /*-------------Claw Control-------------*/
    if (Con.R1 && !Con.last_R1) {
      Piston_Claw.set(!Piston_Claw.value());
    }
    /*-------------Toggle Roller Control-------------*/
    if (Con.R2 && !Con.last_R2) {
      startRolling();
    } else if (!Con.R2) {
      stopRolling();
    }

    if(Con.A && !Con.last_A){
      Piston_Toggle_Lock.set(!Piston_Toggle_Lock.value());
    }
    /*-------------loop sleep time-------------*/
    vex::this_thread::sleep_for(10);
  }
}

void calibrate() {
  timer calibration_timer;
  if (IMU.installed()) {
    while (true) {
      calibration_timer.reset();
      while (true) {
        IMU.calibrate();
        while (IMU.isCalibrating()) {
          this_thread::sleep_for(10);
        }
        if (calibration_timer.time() < 1800) {
          calibration_timer.reset();
        } else {
          break;
        }
      }
      this_thread::sleep_for(200);
      const double prev_heading = IMU.heading(deg);
      this_thread::sleep_for(800);
      if (fabs(IMU.heading(deg) - prev_heading) < 0.1) {
        Controller.rumble(".");
        break;
      }
    }
  } else {
    Controller.rumble("...");
  }
}
