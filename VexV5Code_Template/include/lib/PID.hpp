#pragma once

class PID {
public:
  /**
   * @brief Construct a new PID controller
   *
   * @param _kp proportional gain
   * @param _ki integral gain
   * @param _kd derivative gain
   * @param _windup_range integral anti windup range
   * @param _sign_flip_reset whether to reset integral when sign of error flips
   *
   * @b Example
   * @code {.cpp}
   * // create a PID
   * PID pid(5, // kP
   *         0.01, // kI
   *         20, // kD
   *         5, // integral anti windup range
   *         false); // don't reset integral when sign of error flips
   * @endcode
   */
  PID(float _kp, float _ki, float _kd, float _windup_range = 0, bool _sign_flip_reset = false);

  /**
   * @brief Update the PID
   *
   * @param _error target minus position - AKA error
   * @return float output
   *
   * @b Example
   * @code {.cpp}
   * void opcontrol() {
   *     // create a PID
   *     PID pid(5, 0, 20);
   *     // give the pid a test input
   *     // the pid will then return an output
   *     float output = pid.update(10);
   * }
   * @endcode
   */
  float update(float _error);

  /**
   * @brief reset integral, derivative, and prevTime
   *
   * @b Example
   * @code {.cpp}
   * void opcontrol() {
   *     // create a PID
   *     PID pid(5, 0, 20);
   *     // give the pid a test input
   *     // the pid will then return an output
   *     float output = pid.update(10);
   *     // reset the pid
   *     pid.reset();
   * }
   * @endcode
   */
  void reset();

protected:
  // gains

  const float kp;
  const float ki;
  const float kd;

  // optimizations

  const float windup_range;
  const bool sign_flip_reset;

  float integral = 0;
  float prev_error = 0;
};

/**
 * @brief PID gains struct
 * 
 * @param _kp proportional gain
 * @param _ki integral gain
 * @param _kd derivative gain
 * @param _windup_range integral anti windup range
 *
 */
struct pidGains {
  float kp;
  float ki;
  float kd;
  float windup_range;
};

