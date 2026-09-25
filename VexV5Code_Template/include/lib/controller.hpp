#pragma once

#include "v5.h"
#include "v5_vcs.h"

/**
 * @brief Stores the current and previous state of a VEX controller.
 *
 * 存储 VEX 手柄当前及上一次的状态。
 */
class ControllerStatus {
private:
  const vex::controller* con;

public:
  /**
   * @brief Constructs a status reader for a controller.
   *
   * 为指定手柄构造状态读取器。
   *
   * @param controller Controller to read. / 要读取的手柄。
   */
  explicit ControllerStatus(const vex::controller& controller);

  /** Current L1 button state. / L1 按键当前状态。 */
  bool L1;
  /** Current L2 button state. / L2 按键当前状态。 */
  bool L2;
  /** Current R1 button state. / R1 按键当前状态。 */
  bool R1;
  /** Current R2 button state. / R2 按键当前状态。 */
  bool R2;
  /** Current X button state. / X 按键当前状态。 */
  bool X;
  /** Current Y button state. / Y 按键当前状态。 */
  bool Y;
  /** Current A button state. / A 按键当前状态。 */
  bool A;
  /** Current B button state. / B 按键当前状态。 */
  bool B;
  /** Current left button state. / 左方向键当前状态。 */
  bool LEFT;
  /** Current right button state. / 右方向键当前状态。 */
  bool RIGHT;
  /** Current up button state. / 上方向键当前状态。 */
  bool UP;
  /** Current down button state. / 下方向键当前状态。 */
  bool DOWN;

  /** Previous L1 button state. / L1 按键上一次状态。 */
  bool last_L1;
  /** Previous L2 button state. / L2 按键上一次状态。 */
  bool last_L2;
  /** Previous R1 button state. / R1 按键上一次状态。 */
  bool last_R1;
  /** Previous R2 button state. / R2 按键上一次状态。 */
  bool last_R2;
  /** Previous X button state. / X 按键上一次状态。 */
  bool last_X;
  /** Previous Y button state. / Y 按键上一次状态。 */
  bool last_Y;
  /** Previous A button state. / A 按键上一次状态。 */
  bool last_A;
  /** Previous B button state. / B 按键上一次状态。 */
  bool last_B;
  /** Previous left button state. / 左方向键上一次状态。 */
  bool last_LEFT;
  /** Previous right button state. / 右方向键上一次状态。 */
  bool last_RIGHT;
  /** Previous up button state. / 上方向键上一次状态。 */
  bool last_UP;
  /** Previous down button state. / 下方向键上一次状态。 */
  bool last_DOWN;

  /** Current Axis1 position in percent. / Axis1 当前百分比位置。 */
  int32_t A1;
  /** Current Axis2 position in percent. / Axis2 当前百分比位置。 */
  int32_t A2;
  /** Current Axis3 position in percent. / Axis3 当前百分比位置。 */
  int32_t A3;
  /** Current Axis4 position in percent. / Axis4 当前百分比位置。 */
  int32_t A4;

  /** Previous Axis1 position in percent. / Axis1 上一次百分比位置。 */
  int32_t last_A1;
  /** Previous Axis2 position in percent. / Axis2 上一次百分比位置。 */
  int32_t last_A2;
  /** Previous Axis3 position in percent. / Axis3 上一次百分比位置。 */
  int32_t last_A3;
  /** Previous Axis4 position in percent. / Axis4 上一次百分比位置。 */
  int32_t last_A4;

  /**
   * @brief Samples the controller and advances previous-state values.
   *
   * 采样手柄并更新上一次状态值。
   */
  void update();
};

/**
 * @brief Configures joystick deadzone and response-curve behavior.
 *
 * 配置摇杆死区和响应曲线行为。
 */
struct ControllerCurveParams {
  /** Deadzone magnitude in percent. / 百分比死区大小。 */
  double deadzone;
  /** Minimum nonzero output in percent. / 最小非零输出百分比。 */
  double min_output;
  /** Exponential curve base. / 指数响应曲线的底数。 */
  double curve_exponent;
};

/**
 * @brief Applies configured deadzone and response shaping to joystick input.
 *
 * 对摇杆输入应用配置的死区和响应曲线。
 *
 * @param input Raw joystick input in percent. / 原始摇杆输入百分比。
 * @param params Curve parameters. / 曲线参数。
 * @return Shaped joystick output in percent. / 曲线处理后的输出百分比。
 */
float getControllerCurveOutput(float input, ControllerCurveParams params = {});
