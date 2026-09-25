#pragma once

#include "lib/motor-group.hpp"
#include "v5.h"
#include "v5_vcs.h"

/**
 * @brief Measures linear travel from a motor, motor group, or rotation sensor.
 *
 * 通过电机、电机组或旋转传感器测量直线行程。
 */
class TrackingWheel {
private:
  enum SourceType { MOTOR, MOTOR_GROUP, ROTATION_SENSOR };

  SourceType type;
  vex::motor* motor;
  MotorGroup* motor_group;
  vex::rotation* rotation_sensor;
  double wheel_diameter;
  double gear_ratio;

public:
  /**
   * @brief Constructs a tracking wheel backed by one motor.
   *
   * 构造由单个电机提供数据的里程轮。
   *
   * @param motor Motor used as the rotation source. / 用作旋转数据源的电机。
   * @param wheel_diameter Wheel diameter in the desired distance unit. /
   * 以目标距离单位表示的轮径。
   * @param gear_ratio Wheel rotations per source rotation. /
   * 每次数据源旋转对应的车轮转数。
   */
  TrackingWheel(vex::motor& motor, double wheel_diameter, double gear_ratio);

  /**
   * @brief Constructs a tracking wheel backed by a motor group.
   *
   * 构造由电机组提供数据的里程轮。
   *
   * @param motor_group Motor group used as the rotation source. /
   * 用作旋转数据源的电机组。
   * @param wheel_diameter Wheel diameter in the desired distance unit. /
   * 以目标距离单位表示的轮径。
   * @param gear_ratio Wheel rotations per source rotation. /
   * 每次数据源旋转对应的车轮转数。
   */
  TrackingWheel(MotorGroup& motor_group, double wheel_diameter,
                double gear_ratio);

  /**
   * @brief Constructs a tracking wheel backed by a rotation sensor.
   *
   * 构造由旋转传感器提供数据的里程轮。
   *
   * @param sensor Rotation sensor used as the source. /
   * 用作数据源的旋转传感器。
   * @param wheel_diameter Wheel diameter in the desired distance unit. /
   * 以目标距离单位表示的轮径。
   * @param gear_ratio Wheel rotations per sensor rotation. /
   * 每次传感器旋转对应的车轮转数。
   */
  TrackingWheel(vex::rotation& sensor, double wheel_diameter,
                double gear_ratio = 1.0);

  /**
   * @brief Returns accumulated linear travel.
   *
   * 返回累计直线行程。
   *
   * @return Distance in the same unit as the configured wheel diameter. /
   * 与配置轮径相同单位的距离。
   */
  double getDistance();

  /**
   * @brief Resets the underlying rotation source to zero.
   *
   * 将底层旋转数据源重置为零。
   */
  void reset();
};
