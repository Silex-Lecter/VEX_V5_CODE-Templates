#pragma once

#include "lib/chassis/tracking-wheel.hpp"
#include "lib/controller.hpp"
#include "lib/motor-group.hpp"
#include "lib/pid.hpp"
#include "v5.h"

/**
 * @brief Applies normalized tuning power to one output or an opposing pair.
 *
 * 将归一化调参功率应用到单个输出或方向相反的一对输出。
 */
class TuningOutput {
public:
  /**
   * @brief Constructs an output backed by one motor.
   *
   * 构造由单个电机提供的输出。
   *
   * @param motor Output motor. / 输出电机。
   */
  explicit TuningOutput(vex::motor& motor);

  /**
   * @brief Constructs an output backed by opposing motors.
   *
   * 构造由方向相反的两个电机提供的输出。
   *
   * @param forward Forward-side motor. / 正向侧电机。
   * @param reverse Reverse-side motor. / 反向侧电机。
   */
  TuningOutput(vex::motor& forward, vex::motor& reverse);

  /**
   * @brief Constructs an output backed by one motor group.
   *
   * 构造由单个电机组提供的输出。
   *
   * @param motor_group Output motor group. / 输出电机组。
   */
  explicit TuningOutput(MotorGroup& motor_group);

  /**
   * @brief Constructs an output backed by opposing motor groups.
   *
   * 构造由方向相反的两个电机组提供的输出。
   *
   * @param forward Forward-side motor group. / 正向侧电机组。
   * @param reverse Reverse-side motor group. / 反向侧电机组。
   */
  TuningOutput(MotorGroup& forward, MotorGroup& reverse);

  /**
   * @brief Sets independent forward and reverse output scales.
   *
   * 设置独立的正向和反向输出比例。
   *
   * @param forward Forward-side scale. / 正向侧比例。
   * @param reverse Reverse-side scale. / 反向侧比例。
   * @return This output for chained configuration. / 返回当前输出以便链式配置。
   */
  TuningOutput& setOutputScales(float forward, float reverse = 1.0f);

  /**
   * @brief Writes normalized power to the configured output devices.
   *
   * 向配置的输出设备写入归一化功率。
   *
   * @param power Requested power from `-100` to `100`. / `-100` 到 `100`
   * 的请求功率。
   */
  void write(float power) const;

  /**
   * @brief Stops all configured output devices.
   *
   * 停止全部已配置输出设备。
   */
  void stop() const;

private:
  /**
   * @brief Adapts either one motor or one motor group as a tuning device.
   *
   * 将单个电机或单个电机组适配为调参设备。
   */
  class Device {
  public:
    /**
     * @brief Constructs an empty device.
     *
     * 构造空设备。
     */
    Device();

    /**
     * @brief Constructs a device backed by one motor.
     *
     * 构造由单个电机提供的设备。
     *
     * @param motor Backing motor. / 底层电机。
     */
    explicit Device(vex::motor& motor);

    /**
     * @brief Constructs a device backed by one motor group.
     *
     * 构造由单个电机组提供的设备。
     *
     * @param motor_group Backing motor group. / 底层电机组。
     */
    explicit Device(MotorGroup& motor_group);

    /**
     * @brief Reports whether the device has a backing output.
     *
     * 报告设备是否具有底层输出。
     *
     * @return `true` when a motor or motor group is configured. /
     * 已配置电机或电机组时返回 `true`。
     */
    bool exists() const;

    /**
     * @brief Writes normalized power to the backing output.
     *
     * 向底层输出写入归一化功率。
     *
     * @param power Requested power. / 请求功率。
     */
    void write(float power) const;

    /**
     * @brief Stops the backing output.
     *
     * 停止底层输出。
     */
    void stop() const;

  private:
    vex::motor* motor_;
    MotorGroup* motor_group_;
  };

  Device forward_;
  Device reverse_;
  float forward_scale_;
  float reverse_scale_;
};

/**
 * @brief Defines the feedback interface consumed by the PID tuner.
 *
 * 定义 PID 调参器使用的反馈接口。
 */
class TuningFeedback {
public:
  /**
   * @brief Destroys a feedback source through its interface.
   *
   * 通过接口销毁反馈数据源。
   */
  virtual ~TuningFeedback();

  /**
   * @brief Reads the current feedback value.
   *
   * 读取当前反馈值。
   *
   * @return Current normalized feedback. / 当前归一化反馈值。
   */
  virtual float read() = 0;

  /**
   * @brief Resets the underlying feedback source.
   *
   * 重置底层反馈数据源。
   */
  virtual void reset() = 0;
};

/**
 * @brief Supplies converted motor-position feedback.
 *
 * 提供转换后的电机位置反馈。
 */
class MotorPositionFeedback : public TuningFeedback {
public:
  /**
   * @brief Constructs feedback from one motor.
   *
   * 使用单个电机构造反馈。
   *
   * @param motor Position source. / 位置数据源。
   * @param gear_ratio Output-to-motor position ratio. /
   * 输出位置与电机位置的比率。
   * @param unit_conversion Final unit conversion factor. / 最终单位转换系数。
   */
  MotorPositionFeedback(vex::motor& motor, float gear_ratio = 1.0f,
                        float unit_conversion = 1.0f);

  /**
   * @brief Constructs feedback from a motor group.
   *
   * 使用电机组构造反馈。
   *
   * @param motor_group Position source. / 位置数据源。
   * @param gear_ratio Output-to-motor position ratio. /
   * 输出位置与电机位置的比率。
   * @param unit_conversion Final unit conversion factor. / 最终单位转换系数。
   */
  MotorPositionFeedback(MotorGroup& motor_group, float gear_ratio = 1.0f,
                        float unit_conversion = 1.0f);

  /** @brief Reads converted motor position. / 读取转换后的电机位置。 */
  float read() override;
  /** @brief Resets the backing motor position. / 重置底层电机位置。 */
  void reset() override;

private:
  vex::motor* motor_;
  MotorGroup* motor_group_;
  float gear_ratio_;
  float unit_conversion_;
};

/**
 * @brief Supplies converted tracking-wheel distance feedback.
 *
 * 提供转换后的里程轮距离反馈。
 */
class TrackingWheelFeedback : public TuningFeedback {
public:
  /**
   * @brief Constructs feedback from one tracking wheel.
   *
   * 使用单个里程轮构造反馈。
   *
   * @param tracking_wheel Distance source. / 距离数据源。
   * @param unit_conversion Final unit conversion factor. / 最终单位转换系数。
   */
  explicit TrackingWheelFeedback(TrackingWheel& tracking_wheel,
                                 float unit_conversion = 1.0f);

  /**
   * @brief Constructs averaged feedback from opposing tracking wheels.
   *
   * 使用方向相反的里程轮构造平均反馈。
   *
   * @param forward Forward-side tracking wheel. / 正向侧里程轮。
   * @param reverse Reverse-side tracking wheel. / 反向侧里程轮。
   * @param unit_conversion Final unit conversion factor. / 最终单位转换系数。
   */
  TrackingWheelFeedback(TrackingWheel& forward, TrackingWheel& reverse,
                        float unit_conversion = 1.0f);

  /** @brief Reads converted tracking-wheel distance. / 读取转换后的里程轮距离。
   */
  float read() override;
  /** @brief Resets the backing tracking wheels. / 重置底层里程轮。 */
  void reset() override;

private:
  TrackingWheel& forward_;
  TrackingWheel* reverse_;
  float unit_conversion_;
};

/**
 * @brief Supplies converted inertial-sensor rotation feedback.
 *
 * 提供转换后的惯性传感器旋转反馈。
 */
class InertialRotationFeedback : public TuningFeedback {
public:
  /**
   * @brief Constructs feedback from an inertial sensor.
   *
   * 使用惯性传感器构造反馈。
   *
   * @param inertial Rotation source. / 旋转数据源。
   * @param unit_conversion Final unit conversion factor. / 最终单位转换系数。
   */
  explicit InertialRotationFeedback(vex::inertial& inertial,
                                    float unit_conversion = 1.0f);

  /** @brief Reads converted inertial rotation. / 读取转换后的惯性旋转值。 */
  float read() override;
  /** @brief Resets inertial rotation to zero. / 将惯性旋转值重置为零。 */
  void reset() override;

private:
  vex::inertial& inertial_;
  float unit_conversion_;
};

/**
 * @brief Provides controller-driven interactive PID tuning.
 *
 * 提供由手柄驱动的交互式 PID 调参。
 */
class PidTuner {
public:
  /**
   * @brief Constructs an interactive PID tuner.
   *
   * 构造交互式 PID 调参器。
   *
   * @param output Controlled output. / 被控输出。
   * @param feedback Feedback source. / 反馈数据源。
   * @param controller_status Controller input state. / 手柄输入状态。
   */
  PidTuner(TuningOutput& output, TuningFeedback& feedback,
           ControllerStatus& controller_status);

  /** @brief Sets the proportional-gain adjustment step. /
   * 设置比例增益调整步长。 */
  PidTuner& setKpTuningUnit(float unit);
  /** @brief Sets the derivative-gain adjustment step. / 设置微分增益调整步长。
   */
  PidTuner& setKdTuningUnit(float unit);
  /** @brief Sets the integral-gain adjustment step. / 设置积分增益调整步长。 */
  PidTuner& setKiTuningUnit(float unit);
  /** @brief Sets the windup-range adjustment step. / 设置积分饱和范围调整步长。
   */
  PidTuner& setWindupRangeTuningUnit(float unit);
  /**
   * @brief Sets all gain-adjustment steps.
   *
   * 设置全部增益调整步长。
   *
   * @param kp Proportional step. / 比例增益步长。
   * @param kd Derivative step. / 微分增益步长。
   * @param ki Integral step. / 积分增益步长。
   * @param windup_range Windup-range step. / 积分饱和范围步长。
   * @return This tuner for chained configuration. /
   * 返回当前调参器以便链式配置。
   */
  PidTuner& setTuningUnits(float kp, float kd, float ki, float windup_range);

  /** @brief Sets the test target. / 设置测试目标值。 */
  PidTuner& setTarget(float target);
  /** @brief Sets the test timeout in milliseconds. /
   * 设置以毫秒为单位的测试超时时间。 */
  PidTuner& setTimeout(float timeout_ms);
  /** @brief Replaces the current PID gains. / 替换当前 PID 增益。 */
  PidTuner& setPidGains(pidGains gains);
  /** @brief Returns the current PID gains. / 返回当前 PID 增益。 */
  pidGains getPidGains() const;
  /** @brief Sets the loop delay in milliseconds. / 设置循环延迟时间（毫秒）。
   */
  PidTuner& setLoopDelay(uint32_t loop_delay_ms);
  /**
   * @brief Runs interactive tuning; UP switches between PD and I modes.
   *
   * 运行交互式调参；上方向键用于切换 PD 和 I 模式。
   */
  void runTuning();

private:
  enum class TuningMode { PD, I };

  TuningOutput& output_;
  TuningFeedback& feedback_;
  ControllerStatus& controller_status_;
  pidGains gains_;
  float kp_tuning_unit_;
  float kd_tuning_unit_;
  float ki_tuning_unit_;
  float windup_range_tuning_unit_;
  float target_;
  float timeout_ms_;
  int32_t loop_delay_ms_;

  void runTest();
  void printModeHelp(TuningMode mode) const;
};
