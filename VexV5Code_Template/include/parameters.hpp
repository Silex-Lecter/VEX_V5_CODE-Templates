#pragma once
#include "lib/PID.hpp"

/**
 * @brief IMU 旋转10圈的实际数值，用于修正陀螺仪数据
 */
#define IMU_10 3612.0

/**
 * @brief 底盘前后PID参数
 */
const pidGains BASE_LATERAL_PID{0.27, 0.04, 3.2, 10.0};
/**
 * @brief 底盘旋转PID参数
 */
const pidGains BASE_ANGULAR_PID{2.3, 0.02, 19.0, 2.0};
/**
 * @brief 底盘旋转PID参数（大角度）
 */
const pidGains BASE_ANGULAR_PID_LARGE{1.7, 0.01, 12.5, 2.0};

enum class ArmEvent { RESET, FRONT_BUTTON, FRONT_TOP, TOP, REAR_TOP, REAR_BUTTON };

namespace ArmPosition {
  /**
   * @brief 机械臂位置前低
   */
  const float FRONT_BUTTON = 0;
  /**
   * @brief 机械臂位置前高
   */
  const float FRONT_TOP = 145;
  /**
   * @brief 机械臂位置后高
   */
  const float REAR_TOP = 210;
  /**
   * @brief 机械臂位置后低
   */
  const float REAR_BUTTON = 310;
}