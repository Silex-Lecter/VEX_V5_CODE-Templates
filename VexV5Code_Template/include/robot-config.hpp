#pragma once

#include "api.h"
using namespace vex;

extern brain Brain;

extern controller Controller;
extern ControllerStatus Con;

extern motor Motor_BaseL1;
extern motor Motor_BaseL2;
extern motor Motor_BaseL3;
extern motor Motor_BaseR1;
extern motor Motor_BaseR2;
extern motor Motor_BaseR3;
extern MotorGroup Motor_BaseL;
extern MotorGroup Motor_BaseR;
extern DifferentialChassis Chassis;

extern motor Motor_Elevator1;
extern motor Motor_Elevator2;
extern MotorGroup Motor_Elevator;
extern Arm1DOF Ele_D1;
extern motor Motor_Arm1;
extern motor Motor_Arm2;
extern MotorGroup Motor_Arm;
extern Arm1DOF Arm_D2;

extern motor Motor_ToggleRollerL;
extern motor Motor_ToggleRollerR;
extern MotorGroup Motor_ToggleRoller;

extern pneumatics Piston_Claw;
extern pneumatics Piston_Arm_D3;
extern pneumatics Piston_Toggle_Lock;

extern TrackingWheel Left_Tracking_Wheel;
extern TrackingWheel Right_Tracking_Wheel;
extern inertial IMU;
extern LocalDeltaSolver Solver;
extern Odometry Odom;
extern MotionController ChassisMotion;

extern optical Color_Sensor1;
extern optical Color_Sensor2;
extern RedBlueSensor Toggle_Sensor1;
extern RedBlueSensor Toggle_Sensor2;
extern bumper Elevator_Lower_limit;

/**
 * @brief 机器人设备初始化和自定义类创建（不含陀螺仪初始化）
 */
void initRobot(void);
