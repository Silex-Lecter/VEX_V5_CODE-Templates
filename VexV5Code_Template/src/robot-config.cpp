#include "robot-config.hpp"
#include "api.h"
#include "parameters.hpp"

using namespace vex;

brain Brain;
controller Controller = controller(primary);
ControllerStatus Con = ControllerStatus(Controller);

motor Motor_BaseL1 = motor(PORT2, ratio6_1, true);
motor Motor_BaseL2 = motor(PORT3, ratio18_1, true);
motor Motor_BaseL3 = motor(PORT1, ratio6_1, true);
motor Motor_BaseR1 = motor(PORT10, ratio6_1, false);
motor Motor_BaseR2 = motor(PORT8, ratio18_1, false);//中
motor Motor_BaseR3 = motor(PORT9, ratio6_1, false); //后
MotorGroup Motor_BaseL = MotorGroup(Motor_BaseL1, Motor_BaseL2, Motor_BaseL3);
MotorGroup Motor_BaseR = MotorGroup(Motor_BaseR1, Motor_BaseR2, Motor_BaseR3);
DifferentialChassis Chassis = DifferentialChassis(Motor_BaseL, Motor_BaseR);

motor Motor_Elevator1 = motor(PORT18, ratio18_1, true);
motor Motor_Elevator2 = motor(PORT12, ratio18_1, false);
MotorGroup Motor_Elevator = MotorGroup(Motor_Elevator1, Motor_Elevator2);
Arm1DOF Ele_D1 = Arm1DOF(Motor_Elevator, 4.0 / 360.0 * 16.59 * M_PI,
                         {1.5f, 0.0f, 0.0f, 0.0f});
motor Motor_Arm1 = motor(PORT13, ratio18_1, true);
motor Motor_Arm2 = motor(PORT17, ratio18_1, false);
MotorGroup Motor_Arm = MotorGroup(Motor_Arm1, Motor_Arm2);
Arm1DOF Arm_D2 = Arm1DOF(Motor_Arm, 1 / 3.0f, {2.0f, 0.1f, 18.0f, 2.0f});

motor Motor_ToggleRollerL = motor(PORT11, ratio18_1, true);//侧
motor Motor_ToggleRollerR = motor(PORT20, ratio18_1, true);
MotorGroup Motor_ToggleRoller = MotorGroup(Motor_ToggleRollerL, Motor_ToggleRollerR);

pneumatics Piston_Claw = pneumatics(Brain.ThreeWirePort.F);
pneumatics Piston_Arm_D3 = pneumatics(Brain.ThreeWirePort.G);
pneumatics Piston_Toggle_Lock = pneumatics(Brain.ThreeWirePort.H);

TrackingWheel Left_Tracking_Wheel =
    TrackingWheel(Motor_BaseL, 2.75 * 25.4, 3 / 4.0);
TrackingWheel Right_Tracking_Wheel =
    TrackingWheel(Motor_BaseR, 2.75 * 25.4, 3 / 4.0);
inertial IMU = inertial(PORT16, vex::turnType::left);
LocalDeltaSolver Solver = LocalDeltaSolver(
    Left_Tracking_Wheel, Right_Tracking_Wheel, IMU, 3600 / IMU_10);
Odometry Odom = Odometry(Solver);
MotionController ChassisMotion =
    MotionController(Chassis, Odom, Left_Tracking_Wheel, Right_Tracking_Wheel,
                     {BASE_LATERAL_PID.kp, BASE_LATERAL_PID.ki,
                      BASE_LATERAL_PID.kd, BASE_LATERAL_PID.windup_range},
                     {BASE_ANGULAR_PID.kp, BASE_ANGULAR_PID.ki,
                      BASE_ANGULAR_PID.kd, BASE_ANGULAR_PID.windup_range});

optical Color_Sensor1 = optical(PORT14);
optical Color_Sensor2 = optical(PORT19);
RedBlueSensor Toggle_Sensor1 = RedBlueSensor(Color_Sensor1, 18, 208, 20);
RedBlueSensor Toggle_Sensor2 = RedBlueSensor(Color_Sensor2, 352, 220, 20);

bumper Elevator_Lower_limit = bumper(Brain.ThreeWirePort.E);

bool RemoteControlCodeEnabled = true;

void initRobot(void) {
  Odom.setPose({0, 0, 0});

  Color_Sensor1.setLightPower(100);
  Color_Sensor1.integrationTime(20);
  Color_Sensor1.objectDetectThreshold(150);
  Color_Sensor2.setLightPower(100);
  Color_Sensor2.integrationTime(20);
  Color_Sensor2.objectDetectThreshold(150);
}