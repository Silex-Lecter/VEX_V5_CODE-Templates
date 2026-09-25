#pragma once

/**
 * @brief 在自动路线开始前调用。初始化计时器、自动控制开始前初始化
 */
void auton_init();
/**
 * @brief 显示自动路线时间、运行手动控制前初始化
 */
void auton_pre_usercontrol();
/**
 * @brief 获取自动路线计时器
 * @return int 自动路线计时器的值，单位：毫秒
 */
int getAutonTimer();
/**
 * @brief 循环更改自动路线选择
 */
void changeAutonStrategy();
/**
 * @brief 根据选定的策略运行自动路线
 */
void autonomous();

void auton_left_22();
void auton_right_22();
void auton_left_13();
void auton_right_13();
void auton_left_23();
void auton_right_23();