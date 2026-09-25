#pragma once

#include "lib/motor-group.hpp"
#include "lib/pid.hpp"
#include "v5.h"
#include "v5_vcs.h"

/**
 * @brief Controls a single-degree-of-freedom arm using a motor or motor group.
 *
 * 使用单个电机或电机组控制单自由度机械臂。
 */
class Arm1DOF {
private:
  enum SourceType { MOTOR, MOTOR_GROUP };
  enum MotionType { DISABLE, POSITION, VELOCITY };

  SourceType type;
  MotionType motion_type;
  vex::motor* motor;
  MotorGroup* motor_group;
  float volt_coeff;
  float gear_ratio;
  float slew_rate;
  float max_power;
  PID pid;
  float current_power;
  float prev_power;
  float target;

  void move(float power);
  void move(float power, float slew_rate);
  void stop(vex::brakeType mode = vex::brakeType::coast);

public:
  /**
   * @brief Constructs an arm controlled by one motor.
   *
   * 构造由单个电机控制的机械臂。
   *
   * @param motor Arm motor. / 机械臂电机。
   * @param gear_ratio Output-to-motor position ratio. /
   * 输出位置与电机位置的比率。
   * @param pid_gains Position-controller gains. / 位置控制器增益。
   */
  Arm1DOF(vex::motor& motor, float gear_ratio, pidGains pid_gains);

  /**
   * @brief Constructs an arm controlled by a motor group.
   *
   * 构造由电机组控制的机械臂。
   *
   * @param motor_group Arm motor group. / 机械臂电机组。
   * @param gear_ratio Output-to-motor position ratio. /
   * 输出位置与电机位置的比率。
   * @param pid_gains Position-controller gains. / 位置控制器增益。
   */
  Arm1DOF(MotorGroup& motor_group, float gear_ratio, pidGains pid_gains);

  /**
   * @brief Sets the millivolts-per-power-unit coefficient.
   *
   * 设置每功率单位对应的毫伏系数。
   *
   * @param coefficient Millivolts per power unit. / 每功率单位的毫伏数。
   */
  void setVoltCoeff(float coefficient);

  /**
   * @brief Replaces the position-controller gains and resets its state.
   *
   * 替换位置控制器增益并重置其状态。
   *
   * @param pid_gains New PID gains. / 新的 PID 增益。
   */
  void setPidGains(pidGains pid_gains);

  /**
   * @brief Sets the maximum power change per update.
   *
   * 设置每次更新允许的最大功率变化量。
   *
   * @param slew_rate Maximum power change. / 最大功率变化量。
   */
  void setSlewRate(float slew_rate);

  void setMaxPower(float max_power);
  /**
   * @brief Selects open-loop power control.
   *
   * 选择开环功率控制。
   *
   * @param power Requested power from `-100` to `100`. / `-100` 到 `100`
   * 的请求功率。
   */
  void runAtPower(float power);

  /**
   * @brief Selects PID position control for a target angle.
   *
   * 选择针对目标角度的 PID 位置控制。
   *
   * @param target Target output angle in degrees. / 以度为单位的目标输出角度。
   */
  void moveTo(float target);

  /**
   * @brief Disables active arm motion.
   *
   * 禁用机械臂主动运动。
   */
  void disable();

  /**
   * @brief Returns the arm output angle.
   *
   * 返回机械臂输出角度。
   *
   * @return Output angle in degrees. / 以度为单位的输出角度。
   */
  double deg();

  /**
   * @brief Resets the underlying motor encoder positions.
   *
   * 重置底层电机编码器位置。
   */
  void reset();

  /**
   * @brief Executes one control-loop update.
   *
   * 执行一次控制循环更新。
   */
  void update();
};
