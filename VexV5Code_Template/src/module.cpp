#include "module.hpp"
#include "parameters.hpp"
#include "robot-config.hpp"

static bool robot_color = true;

void changeRobotColor() { changeRobotColor(!robot_color); }

void changeRobotColor(bool color) {
  robot_color = color;
  const RobotColor target_color =
      robot_color ? RobotColor::RED : RobotColor::BLUE;
  Toggle_Sensor1.target_color = target_color;
  Toggle_Sensor2.target_color = target_color;

  Controller.Screen.setCursor(5, 1);
  Controller.Screen.print("%27s", robot_color ? "Red      " : "Blue     ");
}

namespace {
bool is_toggle_ready = false;
bool is_rolling = false;
} // namespace
bool isToggleRolling() { return is_rolling; }
void startRolling() {
  is_toggle_ready = false;
  is_rolling = true;
}
void stopRolling() {
  is_toggle_ready = false;
  is_rolling = false;
}
void toggle_task() {
  bool is_color_correct = false;
  while (true) {
    // check if the detected color is the target color
    if (Toggle_Sensor1.classifyColor() == 1 ||
        Toggle_Sensor2.classifyColor() == 1) {
      is_color_correct = true;
    } else {
      is_color_correct = false;
    }
    // control the toggle roller motor based on the rolling state and color
    // detection
    if (is_rolling) {
      // static float bonus_timer = 0;
      if (is_color_correct) is_toggle_ready = true;
      if (is_toggle_ready && !is_color_correct) {
        Motor_ToggleRoller.stop(vex::brakeType::coast);
        is_rolling = false;
      } else {
        Motor_ToggleRoller.spin(vex::directionType::fwd, 90,
                                vex::velocityUnits::pct);
      }
    } else {
      Motor_ToggleRoller.stop(vex::brakeType::coast);
    }
    // sleep for 10 milliseconds to prevent excessive CPU usage
    vex::this_thread::sleep_for(10);
  }
}

namespace {
ArmEvent arm_event = ArmEvent::FRONT_BUTTON;
}

void setArmLevel(ArmEvent input) { arm_event = input; }
ArmEvent getArmLevel() { return arm_event; }

void arm_task() {
  while (true) {
    switch (arm_event) {
      case ArmEvent::RESET:
        Arm_D2.runAtPower(-40);
        Arm_D2.reset();
        break;
      case ArmEvent::FRONT_BUTTON:
        if (Arm_D2.deg() > 5) {
          Arm_D2.moveTo(ArmPosition::FRONT_BUTTON - 10);
        } else if (Arm_D2.deg() <= 5) {
          Arm_D2.runAtPower(-8);
          Arm_D2.reset();
        } else {
          Arm_D2.runAtPower(0);
        }
        break;
      case ArmEvent::FRONT_TOP:
        Arm_D2.moveTo(ArmPosition::FRONT_TOP);
        break;
      case ArmEvent::TOP:
        if (Arm_D2.deg() < ArmPosition::FRONT_TOP + 20) {
          Arm_D2.runAtPower(40);
        } else {
          Arm_D2.moveTo(ArmPosition::FRONT_TOP + 40);
        };
        break;
      case ArmEvent::REAR_TOP:
        Arm_D2.moveTo(ArmPosition::REAR_TOP);
        break;
      case ArmEvent::REAR_BUTTON:
        Arm_D2.moveTo(ArmPosition::REAR_BUTTON);
        break;
    }

    /*-------------Piston D3 Control-------------*/
    if (Arm_D2.deg() < 120 && arm_event == ArmEvent::FRONT_BUTTON) {
      Piston_Arm_D3.set(false);
    } else if ((Arm_D2.deg() > ArmPosition::FRONT_TOP - 40 &&
                Arm_D2.deg() < ArmPosition::FRONT_TOP + 10 &&
                arm_event == ArmEvent::FRONT_TOP) ||
               arm_event == ArmEvent::TOP) {
      Piston_Arm_D3.set(true);
    } else if (Arm_D2.deg() > ArmPosition::REAR_TOP - 10 &&
               Arm_D2.deg() < ArmPosition::REAR_TOP + 10 &&
               arm_event == ArmEvent::REAR_TOP) {
      Piston_Arm_D3.set(false);
    } else if (Arm_D2.deg() > ArmPosition::REAR_BUTTON - 100 &&
               arm_event == ArmEvent::REAR_BUTTON) {
      Piston_Arm_D3.set(true);
    }

    Arm_D2.update();
    vex::this_thread::sleep_for(10);
  }
}

void resetElevator() {
  vex::timer timer;
  while (!Elevator_Lower_limit.pressing() && timer.time() < 2000) {
    Ele_D1.runAtPower(-70);
    vex::this_thread::sleep_for(20);
  }
  Ele_D1.runAtPower(-1);
  Ele_D1.reset();
}

void tuneChassisAngular() {
  TuningOutput output(Motor_BaseR, Motor_BaseL);
  InertialRotationFeedback feedback(IMU);
  PidTuner tuner(output, feedback, Con);

  tuner.setPidGains(BASE_ANGULAR_PID_LARGE)
      .setTuningUnits(0.1f, 0.01f, 0.5f, 0.5f)
      .setTarget(120.0f)
      .setTimeout(1500.0f)
      .setLoopDelay(5)
      .runTuning();
}

void tuneChassisLateral() {
  MotorGroup chassis_motors(Motor_BaseL1, Motor_BaseL2, Motor_BaseR1,
                            Motor_BaseR2);
  TuningOutput output(chassis_motors);
  TrackingWheelFeedback feedback(Left_Tracking_Wheel, Right_Tracking_Wheel);
  PidTuner tuner(output, feedback, Con);

  tuner.setPidGains(BASE_LATERAL_PID)
      .setTuningUnits(0.01f, 0.01f, 0.1f, 5.0f)
      .setTarget(1200.0f)
      .setLoopDelay(5)
      .runTuning();
}

void tuneArm() {
  TuningOutput output(Motor_Arm1);
  MotorPositionFeedback feedback(Motor_Arm1, 1.0f / 3.0f);
  PidTuner tuner(output, feedback, Con);

  Arm_D2.disable();
  vex::this_thread::sleep_for(20);
  tuner.setPidGains({2.0f, 0.1f, 18.0f, 2.0f})
      .setTuningUnits(0.1f, 0.1f, 1.0f, 1.0f)
      .setTarget(90.0f)
      .setLoopDelay(10)
      .setTimeout(1500)
      .runTuning();
  Arm_D2.disable();
}

void tuneElevator() {
  TuningOutput output(Motor_Elevator);
  MotorPositionFeedback feedback(Motor_Elevator, 2.0f,
                                 16.59f * 3.14159265f / 360.0f);
  PidTuner tuner(output, feedback, Con);

  Ele_D1.disable();
  vex::this_thread::sleep_for(20);
  tuner.setPidGains({3.1f, 0.0f, 0.0f, 0.0f})
      .setTuningUnits(0.1f, 0.01f, 0.1f, 1.0f)
      .setTarget(500.0f)
      .setLoopDelay(10)
      .runTuning();
  Ele_D1.disable();
}
