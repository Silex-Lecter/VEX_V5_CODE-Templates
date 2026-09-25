#pragma once

#include "lib/motor-group.hpp"
#include "v5.h"
#include "v5_vcs.h"

/**
 * @brief Drives a differential chassis from linear and angular power commands.
 *
 * 根据线性和角向功率指令驱动差速底盘。
 */
class DifferentialChassis {
private:
  MotorGroup* leftMotor;
  MotorGroup* rightMotor;
  float volt_coeff;

  float speed2voltage(float speed);

public:
  /** Previous linear power. / 上一次线性功率。 */
  float prev_vx;
  /** Previous angular power. / 上一次角向功率。 */
  float prev_omega;

  /**
   * @brief Constructs a differential chassis from left and right motor groups.
   *
   * 使用左右电机组构造差速底盘。
   *
   * @param left Left motor group. / 左侧电机组。
   * @param right Right motor group. / 右侧电机组。
   */
  DifferentialChassis(MotorGroup& left, MotorGroup& right);

  /**
   * @brief Sets the millivolts-per-power-unit coefficient.
   *
   * 设置每功率单位对应的毫伏系数。
   *
   * @param coefficient Millivolts per power unit. / 每功率单位的毫伏数。
   */
  void setVoltCoefficient(float coefficient);

  /**
   * @brief Applies linear and angular power commands to the chassis.
   *
   * 应用线性和角向功率指令到底盘。
   *
   * @param vx Linear command from `-100` to `100`. / `-100` 到 `100`
   * 的线性指令。
   * @param omega Angular command from `-100` to `100`. / `-100` 到 `100`
   * 的角向指令。
   */
  void move(float vx, float omega);

  /**
   * @brief Stops both sides of the chassis.
   *
   * 停止底盘两侧电机。
   *
   * @param mode Brake mode. / 制动模式。
   */
  void stop(vex::brakeType mode = vex::brakeType::coast);
};
