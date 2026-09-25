#pragma once

#include "lib/chassis/tracking-wheel.hpp"

/**
 * @brief Represents a planar position and heading.
 *
 * 表示平面位置和朝向。
 */
struct Pose2D {
  /** X coordinate. / X 坐标。 */
  double x;
  /** Y coordinate. / Y 坐标。 */
  double y;
  /** Heading in radians. / 以弧度为单位的朝向。 */
  double theta;

  /**
   * @brief Computes Euclidean distance to another pose.
   *
   * 计算到另一个位姿的欧氏距离。
   *
   * @param other Destination pose. / 目标位姿。
   * @return Straight-line distance. / 直线距离。
   */
  double distanceTo(Pose2D other) const;

  /**
   * @brief Computes the global bearing to another pose.
   *
   * 计算指向另一个位姿的全局方位角。
   *
   * @param other Destination pose. / 目标位姿。
   * @return Bearing in radians. / 以弧度为单位的方位角。
   */
  double angleTo(Pose2D other) const;

  /**
   * @brief Returns a pose translated without changing its heading.
   *
   * 返回平移后且朝向不变的位姿。
   *
   * @param dx X-axis offset. / X 轴偏移量。
   * @param dy Y-axis offset. / Y 轴偏移量。
   * @return Translated pose. / 平移后的位姿。
   */
  Pose2D translateBy(double dx, double dy) const;
};

/**
 * @brief Represents a local planar displacement and heading change.
 *
 * 表示局部平面位移和朝向变化。
 */
struct LocalDelta {
  /** Local X displacement. / 局部 X 方向位移。 */
  double dx;
  /** Local Y displacement. / 局部 Y 方向位移。 */
  double dy;
  /** Heading change in radians. / 以弧度为单位的朝向变化。 */
  double dtheta;
};

/**
 * @brief Computes local motion deltas from two tracking wheels and an IMU.
 *
 * 根据两个里程轮和惯性传感器计算局部运动增量。
 */
class LocalDeltaSolver {
private:
  TrackingWheel* left_wheel;
  TrackingWheel* right_wheel;
  vex::inertial* imu;
  double last_left;
  double last_right;
  double last_heading;
  double heading_coeff;

public:
  /**
   * @brief Constructs a local-delta solver.
   *
   * 构造局部运动增量求解器。
   *
   * @param left Left tracking wheel. / 左侧里程轮。
   * @param right Right tracking wheel. / 右侧里程轮。
   * @param imu Inertial sensor used for heading. / 用于朝向测量的惯性传感器。
   * @param heading_coeff IMU heading scale factor. / 惯性传感器朝向缩放系数。
   */
  LocalDeltaSolver(TrackingWheel& left, TrackingWheel& right,
                   vex::inertial& imu, double heading_coeff);

  /**
   * @brief Samples the sensors and returns motion since the previous update.
   *
   * 采样传感器并返回自上次更新后的运动增量。
   *
   * @return Local motion delta. / 局部运动增量。
   */
  LocalDelta update();
};

/**
 * @brief Integrates local motion deltas into a global pose.
 *
 * 将局部运动增量积分为全局位姿。
 */
class Odometry {
private:
  Pose2D pose;
  LocalDeltaSolver* solver;

public:
  /**
   * @brief Constructs odometry at the origin.
   *
   * 在原点构造里程计。
   *
   * @param solver Local-delta source. / 局部运动增量数据源。
   */
  explicit Odometry(LocalDeltaSolver& solver);

  /**
   * @brief Integrates one new local motion sample.
   *
   * 积分一个新的局部运动样本。
   */
  void update();

  /**
   * @brief Replaces the current global pose.
   *
   * 替换当前全局位姿。
   *
   * @param pose New global pose. / 新的全局位姿。
   */
  void setPose(Pose2D pose);

  /**
   * @brief Returns the current global pose.
   *
   * 返回当前全局位姿。
   *
   * @return Current pose. / 当前位姿。
   */
  Pose2D getPose() const;
};
