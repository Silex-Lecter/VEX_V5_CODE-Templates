#pragma once

#include "v5.h"
#include "v5_vcs.h"

/**
 * @brief Identifies the red or blue alliance color.
 *
 * 标识红方或蓝方联盟颜色。
 */
enum class RobotColor {
  RED,  /**< Red alliance color. / 红方联盟颜色。 */
  BLUE, /**< Blue alliance color. / 蓝方联盟颜色。 */
};

/**
 * @brief Classifies optical-sensor readings as target or opposing colors.
 *
 * 将光学传感器读数分类为目标颜色或对方颜色。
 */
class RedBlueSensor {
private:
  vex::optical* sensor;
  float red_hue;
  float blue_hue;
  float error_tolerance;

public:
  /**
   * @brief Constructs a red-blue optical sensor classifier.
   *
   * 构造红蓝光学传感器分类器。
   *
   * @param sensor Optical sensor to read. / 要读取的光学传感器。
   * @param red_hue Calibrated red hue. / 标定后的红色色相。
   * @param blue_hue Calibrated blue hue. / 标定后的蓝色色相。
   * @param error_tolerance Allowed circular hue error. / 允许的环形色相误差。
   */
  RedBlueSensor(vex::optical& sensor, float red_hue, float blue_hue,
                float error_tolerance);

  /** Target alliance color used for classification. / 用于分类的目标联盟颜色。
   */
  RobotColor target_color;

  /**
   * @brief Classifies the currently detected object's color.
   *
   * 对当前检测到的物体颜色进行分类。
   *
   * @return `1` for the target color, `-1` for the opposing color, and `0`
   * for another color or no object. / 目标颜色返回 `1`，对方颜色返回 `-1`，
   * 其他颜色或无物体返回 `0`。
   */
  int classifyColor();
};

/**
 * @brief Exposes a distance sensor as a threshold-based proximity switch.
 *
 * 将距离传感器封装为基于阈值的接近开关。
 */
class ProximitySwitch {
private:
  vex::distance* sensor;
  float threshold;

public:
  /**
   * @brief Constructs a proximity switch.
   *
   * 构造接近开关。
   *
   * @param sensor Distance sensor to read. / 要读取的距离传感器。
   * @param threshold Detection threshold in millimeters. /
   * 以毫米为单位的检测阈值。
   */
  explicit ProximitySwitch(vex::distance& sensor, float threshold = 30.0f);

  /**
   * @brief Reports whether an object is closer than the configured threshold.
   *
   * 报告物体是否比配置的阈值更近。
   *
   * @return `true` when an installed sensor detects a nearby object. /
   * 已安装的传感器检测到近距离物体时返回 `true`。
   */
  bool isNear() const;
};
