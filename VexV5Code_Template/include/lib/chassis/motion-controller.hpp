#pragma once

#include "lib/chassis/differential-chassis.hpp"
#include "lib/chassis/odometry.hpp"
#include "lib/pid.hpp"

/**
 * @brief Selects the commanded direction for a heading turn.
 *
 * 选择转向目标朝向时的指定方向。
 */
enum class AngularDirection {
  CW_CLOCKWISE,         /**< Clockwise. / 顺时针。 */
  CCW_COUNTERCLOCKWISE, /**< Counterclockwise. / 逆时针。 */
  AUTO,                 /**< Shortest direction. / 最短方向。 */
};

/**
 * @brief Configures a distance-based linear move.
 *
 * 配置基于距离的直线运动。
 */
struct MoveByDistanceParams {
  /** Maximum output power. / 最大输出功率。 */
  float max_power = 100.0f;
  /** Minimum nonzero output power. / 最小非零输出功率。 */
  float min_power = 0.0f;
  /** Acceptable distance error. / 可接受的距离误差。 */
  float error_tolerance = 10.0f;
  /** Early exit range in degrees. / 提前退出范围。 */
  float early_exit_range = 0.0f;
};

/**
 * @brief Configures a turn to an absolute heading.
 *
 * 配置转向绝对朝向的运动。
 */
struct TurnToHeadingParams {
  /** Allowed turn direction. / 允许的转动方向。 */
  AngularDirection direction = AngularDirection::AUTO;
  /** Maximum output power. / 最大输出功率。 */
  float max_power = 100.0f;
  /** Minimum nonzero output power. / 最小非零输出功率。 */
  float min_power = 0.0f;
  /** Acceptable heading error in degrees. / 可接受的朝向角度误差。 */
  float error_tolerance = 1.0f;
  /** Chaining range before the target in degrees. / 目标前用于动作衔接的提前退出范围（以度为单位）。 */
  float early_exit_range = 0.0f;
};

/**
 * @brief Configures a move to a planar point.
 *
 * 配置移动到平面目标点的运动。
 */
struct MoveToPointParams {
  /** Whether to drive forward instead of backward. / 是否前进而非后退。 */
  bool forward = true;
  /** Maximum output power. / 最大输出功率。 */
  float max_power = 100.0f;
  /** Minimum nonzero output power. / 最小非零输出功率。 */
  float min_power = 0.0f;
  /** Chaining range before the target. / 目标前用于动作衔接的提前退出范围。 */
  float early_exit_range = 0.0f;
  /** Acceptable position error. / 可接受的位置误差。 */
  float error_tolerance = 10.0f;
};

/**
 * @brief Configures a move to a planar pose.
 *
 * 配置移动到平面目标位姿的运动。
 */
struct MoveToPoseParams {
  /** Whether to drive forward instead of backward. / 是否前进而非后退。 */
  bool forward = true;
  /** Carrot-point lead factor from zero to one. /
   * 零到一范围内的引导点前置系数。 */
  float lead = 0.6f;
  /** Maximum output power. / 最大输出功率。 */
  float max_power = 100.0f;
  /** Minimum nonzero output power. / 最小非零输出功率。 */
  float min_power = 0.0f;
  /** Chaining range before the target. / 目标前用于动作衔接的提前退出范围。 */
  float early_exit_range = 0.0f;
  /** Acceptable position error. / 可接受的位置误差。 */
  float error_tolerance = 10.0f;
};

/**
 * @brief Executes closed-loop motion commands for a differential chassis.
 *
 * 为差速底盘执行闭环运动指令。
 */
class MotionController {
private:
  DifferentialChassis* chassis;
  Odometry* odometry;
  TrackingWheel* left_tracking_wheel;
  TrackingWheel* right_tracking_wheel;
  pidGains lateral_pid_gains;
  pidGains angular_pid_gains;

  float getX();

public:
  /**
   * @brief Constructs a chassis motion controller.
   *
   * 构造底盘运动控制器。
   *
   * @param chassis Differential chassis to command. / 要控制的差速底盘。
   * @param odometry Pose estimator. / 位姿估计器。
   * @param left Left tracking wheel. / 左侧里程轮。
   * @param right Right tracking wheel. / 右侧里程轮。
   * @param lateral_pid_gains Linear-motion PID gains. / 直线运动 PID 增益。
   * @param angular_pid_gains Angular-motion PID gains. / 角向运动 PID 增益。
   */
  MotionController(DifferentialChassis& chassis, Odometry& odometry,
                   TrackingWheel& left, TrackingWheel& right,
                   pidGains lateral_pid_gains, pidGains angular_pid_gains);

  /**
   * @brief Sets linear-motion PID gains.
   *
   * 设置直线运动 PID 增益。
   *
   * @param gains New gains. / 新增益。
   */
  void setLateralPIDGains(pidGains gains);

  /**
   * @brief Sets angular-motion PID gains.
   *
   * 设置角向运动 PID 增益。
   *
   * @param gains New gains. / 新增益。
   */
  void setAngularPIDGains(pidGains gains);

  /**
   * @brief Drives at fixed linear and angular power for a duration.
   *
   * 在指定时长内以固定线性和角向功率驱动。
   *
   * @param vx Linear power command. / 线性功率指令。
   * @param omega Angular power command. / 角向功率指令。
   * @param time_ms Duration in milliseconds. / 以毫秒为单位的持续时间。
   */
  void moveForTime(float vx, float omega, float time_ms);

  /**
   * @brief Moves a requested linear distance while holding a heading.
   *
   * 在保持朝向的同时移动指定直线距离。
   *
   * @param distance Requested distance. / 请求距离。
   * @param target_heading Target heading in degrees. / 以度为单位的目标朝向。
   * @param timeout_ms Timeout in milliseconds. / 以毫秒为单位的超时时间。
   * @param params Motion parameters. / 运动参数。
   */
  void moveByDistance(float distance, float target_heading,
                      int timeout_ms = 3000, MoveByDistanceParams params = {});

  /**
   * @brief Turns to an absolute heading.
   *
   * 转向绝对目标朝向。
   *
   * @param degree Target heading in degrees. / 以度为单位的目标朝向。
   * @param timeout_ms Timeout in milliseconds. / 以毫秒为单位的超时时间。
   * @param params Motion parameters. / 运动参数。
   */
  void turnToHeading(float degree, int timeout_ms = 1500,
                     TurnToHeadingParams params = {});

  /**
   * @brief Moves to a global point.
   *
   * 移动到全局目标点。
   *
   * @param x Target X coordinate. / 目标 X 坐标。
   * @param y Target Y coordinate. / 目标 Y 坐标。
   * @param timeout_ms Timeout in milliseconds. / 以毫秒为单位的超时时间。
   * @param params Motion parameters. / 运动参数。
   */
  void moveToPoint(float x, float y, int timeout_ms = 3000,
                   MoveToPointParams params = {});

  /**
   * @brief Moves to a global pose.
   *
   * 移动到全局目标位姿。
   *
   * @param target Target pose. / 目标位姿。
   * @param timeout_ms Timeout in milliseconds. / 以毫秒为单位的超时时间。
   * @param params Motion parameters. / 运动参数。
   */
  void moveToPose(Pose2D target, int timeout_ms = 3000,
                  MoveToPoseParams params = {});
};
