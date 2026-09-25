#pragma once

void enableUsercontrol();

/**
 * @brief
 * 手动控制函数，在此函数中实现了手动控制的逻辑，包括底盘控制、电梯控制和机械臂控制等。
 *
 */
void usercontrol(void);

/**
 * @brief 初始化陀螺仪和定位系统
 *
 */
void calibrate();