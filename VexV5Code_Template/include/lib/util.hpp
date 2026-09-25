#pragma once

#include <cmath>

/**
 * @brief Return the sign of a number
 *
 * @param _input the number to get the sign of
 * @return int -1 if negative, 1 if positive
 *
 * @b Example
 * @code{.cpp}
 * sgn(-10); // returns -1
 * sgn(10); // returns 1
 * sgn(0); // returns 1 (by convention)
 * @endcode
 */
template <typename T> constexpr T sign(T _input) {
  return (_input < 0) ? -1 : 1;
}

/**
 * @brief Slew rate limiter
 *
 * @param _target target value
 * @param _current current value
 * @param _max_change maximum change. No maximum if set to 0

 * @return float the limited value
 *
 * @b Example
 * @code{.cpp}
 * float limited = slew(100, 0, 10);
 * // limited == 10
 * float limited2 = slew(4, 0, 10);
 * // limited2 == 4
 * @endcode
 */
float slew(float _target, float _current, float _max_change);

/**
 * @brief Clamp a value between a minimum and maximum
 *
 * @param _value
 * @param _min
 * @param _max
 * @return const T& clamped value
 */
template <class T>
constexpr const T& clamp(const T& _value, const T& _min, const T& _max) {
  return (_value < _min) ? _min : (_max < _value) ? _max : _value;
}

/**
 * @brief Convert degrees to radians
 *
 * @return constexpr float angle in radians
 */
constexpr float deg2rad(float _deg) { return _deg / 180.0 * M_PI; }

/**
 * @brief Convert radians to degrees
 *
 * @return constexpr float angle in degrees
 */
constexpr float rad2deg(float _rad) { return _rad / M_PI * 180.0; }

/**
 * @brief Normalize an angle in degrees to the range [0, 360]
 *
 * @param _angle angle in degrees
 * @return constexpr float normalized angle in degrees
 */
float normalizeDegrees(float _angle);
