#pragma once

// =============================================================================
// Robot SAVO — savo_control / control_math.hpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Reusable math helpers for Robot Savo control nodes and controllers.
//
// This header provides small, ROS-independent utility functions commonly needed
// in motion control and PID/recovery logic, such as:
//   - clamping / saturation
//   - sign helpers
//   - deadband
//   - near-zero checks
//   - angle normalization (wrap to [-pi, pi])
//   - shortest angular distance
//   - finite-value validation
//
// Design note
// -----------
// Keep these helpers independent from ROS messages and node code.
// Nodes should convert ROS messages to plain values, call these helpers,
// and then publish outputs. This improves reuse, testability, and clarity.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace savo_control
{

class ControlMath
{
public:
  // ---------------------------------------------------------------------------
  // Constants
  // ---------------------------------------------------------------------------
  static constexpr double kPi    = 3.1415926535897932384626433832795;
  static constexpr double kTwoPi = 2.0 * kPi;

  // ---------------------------------------------------------------------------
  // Generic clamp / saturation
  // ---------------------------------------------------------------------------
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
    static_assert(std::is_arithmetic<T>::value, "ControlMath::saturate_abs requires arithmetic type");
    const T lim = (abs_limit < static_cast<T>(0)) ? -abs_limit : abs_limit;
    return clamp<T>(value, -lim, lim);
  }

  // ---------------------------------------------------------------------------
  // Sign helpers
  // ---------------------------------------------------------------------------
  template<typename T>
  static constexpr int signum(const T value)
  {
    static_assert(std::is_arithmetic<T>::value, "ControlMath::signum requires arithmetic type");
    return (T(0) < value) - (value < T(0));
  }

  template<typename T>
  static constexpr T copy_sign(const T magnitude, const T sign_source)
  {
    static_assert(std::is_arithmetic<T>::value, "ControlMath::copy_sign requires arithmetic type");
    return (sign_source < static_cast<T>(0))
      ? -std::abs(magnitude)
      :  std::abs(magnitude);
  }

  // ---------------------------------------------------------------------------
  // Deadband / epsilon checks
  // ---------------------------------------------------------------------------
  static inline double apply_deadband(double value, double deadband)
  {
    const double db = std::abs(deadband);
    if (db <= 0.0) {
      return value;
    }
    return (std::abs(value) < db) ? 0.0 : value;
  }

  static inline bool near_zero(double value, double epsilon = 1e-9)
  {
    return std::abs(value) <= std::abs(epsilon);
  }

  static inline bool nearly_equal(double a, double b, double epsilon = 1e-9)
  {
    return std::abs(a - b) <= std::abs(epsilon);
  }

  // ---------------------------------------------------------------------------
  // Finite / valid checks
  // ---------------------------------------------------------------------------
  static inline bool is_finite(double value)
  {
    return std::isfinite(value);
  }

  static inline bool is_positive_finite(double value)
  {
    return std::isfinite(value) && (value > 0.0);
  }

  static inline bool valid_dt(double dt_sec, double min_dt = 1e-6, double max_dt = 1.0)
  {
    return std::isfinite(dt_sec) && (dt_sec >= min_dt) && (dt_sec <= max_dt);
  }

  // ---------------------------------------------------------------------------
  // Linear interpolation / mapping helpers
  // ---------------------------------------------------------------------------
  static inline double lerp(double a, double b, double t)
  {
    return a + (b - a) * t;
  }

  static inline double inv_lerp(double a, double b, double value)
  {
    const double denom = (b - a);
    if (near_zero(denom)) {
      return 0.0;
    }
    return (value - a) / denom;
  }

  static inline double remap(
    double in_value,
    double in_min,
    double in_max,
    double out_min,
    double out_max,
    bool clamp_input = true)
  {
    if (near_zero(in_max - in_min)) {
      return out_min;
    }

    double x = in_value;
    if (clamp_input) {
      x = clamp(x, std::min(in_min, in_max), std::max(in_min, in_max));
    }

    const double t = (x - in_min) / (in_max - in_min);
    return out_min + t * (out_max - out_min);
  }

  // ---------------------------------------------------------------------------
  // Angle helpers (radians)
  // ---------------------------------------------------------------------------
  // Wrap any angle to [-pi, pi]
  static inline double wrap_angle_rad(double angle_rad)
  {
    if (!std::isfinite(angle_rad)) {
      return 0.0;
    }

    // std::fmod keeps sign; normalize first to (-2pi, 2pi)
    angle_rad = std::fmod(angle_rad + kPi, kTwoPi);
    if (angle_rad < 0.0) {
      angle_rad += kTwoPi;
    }
    return angle_rad - kPi;
  }

  // Wrap angle to [0, 2pi)
  static inline double wrap_angle_2pi_rad(double angle_rad)
  {
    if (!std::isfinite(angle_rad)) {
      return 0.0;
    }

    angle_rad = std::fmod(angle_rad, kTwoPi);
    if (angle_rad < 0.0) {
      angle_rad += kTwoPi;
    }
    return angle_rad;
  }

  // Returns the signed shortest angular distance from 'from' to 'to' in [-pi, pi]
  // Positive -> rotate CCW, Negative -> rotate CW
  static inline double shortest_angular_distance_rad(double from_rad, double to_rad)
  {
    return wrap_angle_rad(to_rad - from_rad);
  }

  // ---------------------------------------------------------------------------
  // Safe division helpers
  // ---------------------------------------------------------------------------
  static inline double safe_div(double numerator, double denominator, double fallback = 0.0)
  {
    if (!std::isfinite(numerator) || !std::isfinite(denominator) || near_zero(denominator)) {
      return fallback;
    }
    return numerator / denominator;
  }

  // ---------------------------------------------------------------------------
  // Slew/step helpers
  // ---------------------------------------------------------------------------
  // Move current toward target by at most max_step (absolute) in one update.
  static inline double step_toward(double current, double target, double max_step)
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

  // ---------------------------------------------------------------------------
  // Unit helpers (optional convenience)
  // ---------------------------------------------------------------------------
  static inline double deg_to_rad(double deg)
  {
    return deg * (kPi / 180.0);
  }

  static inline double rad_to_deg(double rad)
  {
    return rad * (180.0 / kPi);
  }
};

}  // namespace savo_control