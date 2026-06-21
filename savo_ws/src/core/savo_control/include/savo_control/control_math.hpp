#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace savo_control
{

class ControlMath
{
public:
  static constexpr double kPi = 3.1415926535897932384626433832795;
  static constexpr double kTwoPi = 2.0 * kPi;

  template<typename T>
  static constexpr T clamp(const T value, T min_value, T max_value)
  {
    static_assert(std::is_arithmetic<T>::value, "ControlMath::clamp requires arithmetic type");

    if (min_value > max_value) {
      const T tmp = min_value;
      min_value = max_value;
      max_value = tmp;
    }

    return (value < min_value) ? min_value : ((value > max_value) ? max_value : value);
  }

  template<typename T>
  static constexpr T saturate_abs(const T value, const T abs_limit)
  {
    static_assert(
      std::is_arithmetic<T>::value,
      "ControlMath::saturate_abs requires arithmetic type");

    const T limit = (abs_limit < static_cast<T>(0)) ? -abs_limit : abs_limit;
    return clamp<T>(value, -limit, limit);
  }

  template<typename T>
  static constexpr T clamp_abs(const T value, const T abs_limit)
  {
    return saturate_abs<T>(value, abs_limit);
  }

  template<typename T>
  static constexpr int signum(const T value)
  {
    static_assert(
      std::is_arithmetic<T>::value,
      "ControlMath::signum requires arithmetic type");

    return (static_cast<T>(0) < value) - (value < static_cast<T>(0));
  }

  template<typename T>
  static constexpr T copy_sign(const T magnitude, const T sign_source)
  {
    static_assert(
      std::is_arithmetic<T>::value,
      "ControlMath::copy_sign requires arithmetic type");

    const T mag = (magnitude < static_cast<T>(0)) ? -magnitude : magnitude;
    return (sign_source < static_cast<T>(0)) ? -mag : mag;
  }

  static inline bool is_finite(const double value)
  {
    return std::isfinite(value);
  }

  static inline double finite_or_zero(const double value)
  {
    return std::isfinite(value) ? value : 0.0;
  }

  static inline bool is_positive_finite(const double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  static inline double apply_deadband(const double value, const double deadband)
  {
    const double db = std::abs(deadband);
    if (db <= 0.0 || !std::isfinite(db)) {
      return value;
    }

    return (std::abs(value) < db) ? 0.0 : value;
  }

  static inline bool near_zero(const double value, const double epsilon = 1.0e-9)
  {
    return std::abs(value) <= std::abs(epsilon);
  }

  static inline bool nearly_equal(
    const double a,
    const double b,
    const double epsilon = 1.0e-9)
  {
    return std::abs(a - b) <= std::abs(epsilon);
  }

  static inline bool within_tolerance(
    const double value,
    const double target,
    const double tolerance)
  {
    return std::abs(value - target) <= std::abs(tolerance);
  }

  static inline bool valid_dt(
    const double dt_sec,
    const double min_dt = 1.0e-6,
    const double max_dt = 1.0)
  {
    const double safe_min = (std::isfinite(min_dt) && min_dt > 0.0) ? min_dt : 1.0e-6;
    const double safe_max = (std::isfinite(max_dt) && max_dt >= safe_min) ? max_dt : 1.0;

    return std::isfinite(dt_sec) && dt_sec >= safe_min && dt_sec <= safe_max;
  }

  static inline double hypot2(const double x, const double y)
  {
    return std::hypot(finite_or_zero(x), finite_or_zero(y));
  }

  static inline double lerp(const double a, const double b, const double t)
  {
    return a + (b - a) * t;
  }

  static inline double inv_lerp(const double a, const double b, const double value)
  {
    const double denominator = b - a;
    if (!std::isfinite(denominator) || near_zero(denominator)) {
      return 0.0;
    }

    return (value - a) / denominator;
  }

  static inline double remap(
    const double in_value,
    const double in_min,
    const double in_max,
    const double out_min,
    const double out_max,
    const bool clamp_input = true)
  {
    if (near_zero(in_max - in_min)) {
      return out_min;
    }

    double value = in_value;
    if (clamp_input) {
      value = clamp(value, std::min(in_min, in_max), std::max(in_min, in_max));
    }

    const double t = (value - in_min) / (in_max - in_min);
    return out_min + t * (out_max - out_min);
  }

  static inline double wrap_angle_rad(double angle_rad)
  {
    if (!std::isfinite(angle_rad)) {
      return 0.0;
    }

    angle_rad = std::fmod(angle_rad + kPi, kTwoPi);
    if (angle_rad < 0.0) {
      angle_rad += kTwoPi;
    }

    double wrapped = angle_rad - kPi;

    if (nearly_equal(wrapped, -kPi, 1.0e-12)) {
      wrapped = kPi;
    }

    if (near_zero(wrapped, 1.0e-15)) {
      wrapped = 0.0;
    }

    return wrapped;
  }

  static inline double wrap_angle_2pi_rad(double angle_rad)
  {
    if (!std::isfinite(angle_rad)) {
      return 0.0;
    }

    angle_rad = std::fmod(angle_rad, kTwoPi);
    if (angle_rad < 0.0) {
      angle_rad += kTwoPi;
    }

    if (near_zero(angle_rad, 1.0e-15) || nearly_equal(angle_rad, kTwoPi, 1.0e-12)) {
      return 0.0;
    }

    return angle_rad;
  }

  static inline double shortest_angular_distance_rad(
    const double from_rad,
    const double to_rad)
  {
    return wrap_angle_rad(to_rad - from_rad);
  }

  static inline double safe_div(
    const double numerator,
    const double denominator,
    const double fallback = 0.0)
  {
    if (
      !std::isfinite(numerator) ||
      !std::isfinite(denominator) ||
      near_zero(denominator))
    {
      return fallback;
    }

    return numerator / denominator;
  }

  static inline double safe_divide(
    const double numerator,
    const double denominator,
    const double fallback = 0.0)
  {
    return safe_div(numerator, denominator, fallback);
  }

  static inline double step_toward(
    const double current,
    const double target,
    const double max_step)
  {
    const double step = std::abs(max_step);
    if (step <= 0.0 || !std::isfinite(step)) {
      return target;
    }

    const double delta = target - current;
    if (delta > step) {
      return current + step;
    }
    if (delta < -step) {
      return current - step;
    }

    return target;
  }

  static inline double deg_to_rad(const double deg)
  {
    return std::isfinite(deg) ? deg * (kPi / 180.0) : 0.0;
  }

  static inline double rad_to_deg(const double rad)
  {
    return std::isfinite(rad) ? rad * (180.0 / kPi) : 0.0;
  }
};

}  // namespace savo_control
