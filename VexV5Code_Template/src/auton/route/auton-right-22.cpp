#include "api.h"
#include "autonomous.hpp"
#include "module.hpp"
#include "parameters.hpp"
#include "robot-config.hpp"

void auton_right_22() {
  TurnToHeadingParams turn_to_params = {.error_tolerance = 1, .min_power = 9};
  auton_init();
  Odom.setPose({0, 0, Odom.getPose().theta});

  // toggle
  Chassis.move(0, -60);
  startRolling();
  while (isToggleRolling() && getAutonTimer() < 3000) {
    if (rad2deg(Odom.getPose().theta) <-2) {
      Chassis.move(-10, 0);
    }
    vex::this_thread::sleep_for(10);
  }

  // put preload
  printf("\n=put preload=\n");
  Ele_D1.moveTo(25);
  Odom.setPose({0, 0, Odom.getPose().theta});
  ChassisMotion.moveToPose({300, 220, deg2rad(65)}, 1500,
                           {.min_power = 25, .lead = 0.65});
  Chassis.move(35, 0);
  Ele_D1.runAtPower(-65);
  vex::this_thread::sleep_for(100);
  printf("heading: %f\n", rad2deg(Odom.getPose().theta));
  Piston_Claw.set(false);
  vex::this_thread::sleep_for(50);
  Ele_D1.runAtPower(-5);

   // take pin+cup beside the wall
  printf("\n=take pin+cup beside the wall=\n");
  Odom.setPose({0, 0, Odom.getPose().theta});
  ChassisMotion.moveByDistance(-180, 65, 800, {.min_power = 10});
  ChassisMotion.turnToHeading(100, 1500,
                              {.min_power = 30, .early_exit_range = 10});
  ChassisMotion.moveToPose({-220, 50, deg2rad(150)}, 1500,
                           {.min_power = 25, .lead = 0.45});
 
  this_thread::sleep_for(300);
  Piston_Claw.set(true);
  this_thread::sleep_for(100);
  Ele_D1.moveTo(300);
  this_thread::sleep_for(50);

  // put pin+cup beside the wall
  printf("\n=put pin+cup beside the wall=\n");
  ChassisMotion.turnToHeading(120, 1500,
                              {.min_power = 30, .early_exit_range = 10});
  ChassisMotion.moveToPoint(-80, -220, 1500,
                            {.min_power = 10, .forward = false});
  ChassisMotion.turnToHeading(72, 1500,
                              {.min_power = 30, .early_exit_range = 10});
  ChassisMotion.moveByDistance(180, 72, 800, {.min_power = 10});
  vex::this_thread::sleep_for(200);
  Ele_D1.moveTo(190);
  vex::this_thread::sleep_for(100);
  Piston_Claw.set(false);
  printf("heading: %f\n", rad2deg(Odom.getPose().theta));

  // go to neutral goal
  printf("\n=go to neutral goal=\n");
  ChassisMotion.moveByDistance(-300, 72, 1000);
  ChassisMotion.setAngularPIDGains(BASE_ANGULAR_PID_LARGE);
  ChassisMotion.turnToHeading(-80, 1500, turn_to_params);
  ChassisMotion.setAngularPIDGains(BASE_ANGULAR_PID);
  ChassisMotion.moveByDistance(100, -80, 600, {.min_power = 35});
  vex::this_thread::sleep_for(200);

  // take second pin+cup beside the wall
  printf("\n=take second pin+cup beside the wall=\n");
  Odom.setPose({0, 0, Odom.getPose().theta});
  Ele_D1.moveTo(0);
   resetElevator();
   ChassisMotion.turnToHeading(-100, 1500,
                               {.min_power = 30, .early_exit_range = 10});
    ChassisMotion.moveToPose({-130, -260, deg2rad(-155)}, 1500,
                            {.min_power = 25, .lead = 0.45});
     vex::this_thread::sleep_for(200);                      
  
  Piston_Claw.set(true);
  this_thread::sleep_for(100);
   Ele_D1.moveTo(30);
  ChassisMotion.turnToHeading(-120, 1500,
                               {.min_power = 30, .early_exit_range = 10});
   ChassisMotion.moveByDistance(-300, -120, 1000);
   Ele_D1.moveTo(480);
  ChassisMotion.turnToHeading(-80, 1500,
                              {.min_power = 30, .early_exit_range = 10});
 ChassisMotion.moveByDistance(210, -80, 1000);
   Ele_D1.moveTo(270);
   Piston_Claw.set(false);
    vex::this_thread::sleep_for(100);
   ChassisMotion.moveByDistance(-200, -80, 1000);
  auton_pre_usercontrol();
}