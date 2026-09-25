#pragma once
#include "parameters.hpp"

/**
 * @brief change robot color between red and blue
 */
void changeRobotColor();

/**
 * @brief set robot color between red and blue
 *
 * @param _color 1=red, 0=blue
 */
void changeRobotColor(bool _color);

bool isToggleRolling();
void startRolling();
void stopRolling();
void toggle_task();

void setArmLevel(ArmEvent input);
ArmEvent getArmLevel();
void arm_task();

void resetElevator();

void tuneChassisAngular();
void tuneChassisLateral();
void tuneArm();
void tuneElevator();
