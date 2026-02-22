#pragma once

// =============================================================================
// Robot SAVO — savo_control / command_limiter.hpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Reusable command limiting utility for motion commands in `savo_control`.
// It is designed for the cmd_vel shaping stage and can be reused by test,
// PID, and recovery nodes.
//
// What it does
// ------------
// - Axis-wise value clamping        (vx, vy, omega)
// - Axis-wise deadband              (optional zeroing near 0)
// - Slew-rate limiting              (accel/decel style limiting per update)
// - Optional timeout-safe reset helper support (via reset())
// - Stateless helpers + stateful per-axis limiter
//
// Design note
// -----------
// This header is intentionally lightweight and reusable. ROS message handling
// should stay in nodes (e.g., cmd_vel_shaper_node.cpp). This utility focuses on
// pure control/math behavior so it is easy to test and reuse.
//
// Professional usage pattern
// --------------------------
// 1) Configure limits once from parameters
// 2) On each control loop tick:
//      - sanitize dt
//      - clamp target command
//      - apply deadband
//      - apply slew-rate limit from previous output
// 3) Publish shaped command
// =============================================================================

#include <algorithm>
#include <cmath>
#include <limits>

namespace savo_control
{

// -----------------------------------------------------------------------------
// Basic axis configuration
// -----------------------------------------------------------------------------
struct AxisLimitConfig
{
  // Absolute output bounds (final command clamp)
  double min_value { -1.0 };
  double max_value {  1.0 };

  // Deadband around zero (|x| < deadband -> 0)
  double deadband { 0.0 };

  // Slew-rate limits (units per second)
  // Example:
  //   linear x/y  -> m/s^2 equivalent (change in m/s per second)
  //   angular z   -> rad/s^2 equivalent (change in rad/s per second)
  //
  // "rise" = increasing magnitude in positive direction (or from smaller to larger)
  // "fall" = decreasing in negative direction (or from larger to smaller)
  //
  // If <= 0, rate limiting for that direction is effectively disabled.
  double max_rise_rate { 0.0 };
  double max_fall_rate { 0.0 };

  // Optional symmetric helper (not required to use)
  void set_symmetric_bounds(double abs_max)
  {
    const double a = std::abs(abs_max);
    min_value = -a;
    max_value =  a;
  }

  void set_symmetric_rate(double rate)
  {
    const double r = std::abs(rate);
    max_rise_rate = r;
    max_fall_rate = r;
  }
};

// -----------------------------------------------------------------------------
// 3-axis command configuration (vx, vy, omega)
// -----------------------------------------------------------------------------
struct CommandLimiterConfig
{
  AxisLimitConfig vx {};
  AxisLimitConfig vy {};
  AxisLimitConfig wz {};
};

// -----------------------------------------------------------------------------
// Command container (ROS-independent)
// -----------------------------------------------------------------------------
struct MotionCommand
{
  double vx {0.0};   // linear.x  [m/s]
  double vy {0.0};   // linear.y  [m/s] (mecanum/holonomic)
  double wz {0.0};   // angular.z [rad/s]
};

// -----------------------------------------------------------------------------
// Utility helpers (stateless)
// -----------------------------------------------------------------------------
class CommandLimiterMath
{
public:
  static inline double clamp(double value, double min_value, double max_value)
  {
    if (min_value > max_value) {
      std::swap(min_value, max_value);
    }
    return std::clamp(value, min_value, max_value);
  }

  static inline double apply_deadband(double value, double deadband)
  {
    const double db = std::abs(deadband);
    if (db <= 0.0) {
      return value;
    }
    return (std::abs(value) < db) ? 0.0 : value;
  }

  // Slew-rate limit one scalar using previous output and dt.
  // If rate <= 0 or dt <= 0, returns target (no rate limiting).
  static inline double slew_limit(
    double target,
    double previous_output,
    double max_rise_rate,
    double max_fall_rate,
    double dt_sec)
  {
    if (!(dt_sec > 0.0) || !std::isfinite(dt_sec)) {
      return target;
    }

    const double delta = target - previous_output;

    // Select rate by delta direction
    if (delta > 0.0) {
      const double r = std::abs(max_rise_rate);
      if (r <= 0.0 || !std::isfinite(r)) {
        return target;
      }
      const double max_step = r * dt_sec;
      return previous_output + std::min(delta, max_step);
    } else if (delta < 0.0) {
      const double r = std::abs(max_fall_rate);
      if (r <= 0.0 || !std::isfinite(r)) {
        return target;
      }
      const double max_step = r * dt_sec;
      return previous_output + std::max(delta, -max_step);
    }

    return target;
  }

  // Convenience one-axis full pipeline:
  // clamp -> deadband -> slew limit -> clamp again
  static inline double limit_axis(
    double target,
    double previous_output,
    const AxisLimitConfig & cfg,
    double dt_sec)
  {
    double out = target;

    // First clamp target
    out = clamp(out, cfg.min_value, cfg.max_value);

    // Deadband before slew (prevents tiny command chatter)
    out = apply_deadband(out, cfg.deadband);

    // Slew limit against previous shaped output
    out = slew_limit(out, previous_output, cfg.max_rise_rate, cfg.max_fall_rate, dt_sec);

    // Final clamp for safety
    out = clamp(out, cfg.min_value, cfg.max_value);

    // Final deadband cleanup
    out = apply_deadband(out, cfg.deadband);

    return out;
  }
};

// -----------------------------------------------------------------------------
// Stateful command limiter (stores previous output command)
// -----------------------------------------------------------------------------
class CommandLimiter
{
public:
  CommandLimiter() = default;
  explicit CommandLimiter(const CommandLimiterConfig & cfg)
  : config_(cfg)
  {}

  void set_config(const CommandLimiterConfig & cfg)
  {
    config_ = cfg;
  }

  const CommandLimiterConfig & config() const
  {
    return config_;
  }

  // Reset internal state (e.g., on mode switch, timeout, emergency stop, startup)
  void reset()
  {
    prev_output_ = MotionCommand{};
    has_prev_output_ = false;
  }

  // Reset to a known command state
  void reset_to(const MotionCommand & cmd)
  {
    prev_output_ = cmd;
    has_prev_output_ = true;
  }

  bool has_previous_output() const
  {
    return has_prev_output_;
  }

  const MotionCommand & previous_output() const
  {
    return prev_output_;
  }

  // Main limiter call
  // If there is no previous output yet, it uses zero as previous baseline.
  MotionCommand limit(const MotionCommand & target, double dt_sec)
  {
    const MotionCommand baseline = has_prev_output_ ? prev_output_ : MotionCommand{};

    MotionCommand out;
    out.vx = CommandLimiterMath::limit_axis(target.vx, baseline.vx, config_.vx, dt_sec);
    out.vy = CommandLimiterMath::limit_axis(target.vy, baseline.vy, config_.vy, dt_sec);
    out.wz = CommandLimiterMath::limit_axis(target.wz, baseline.wz, config_.wz, dt_sec);

    prev_output_ = out;
    has_prev_output_ = true;
    return out;
  }

  // Stateless-style helpers exposed through the class for convenience
  static MotionCommand clamp_only(const MotionCommand & in, const CommandLimiterConfig & cfg)
  {
    MotionCommand out;
    out.vx = CommandLimiterMath::clamp(in.vx, cfg.vx.min_value, cfg.vx.max_value);
    out.vy = CommandLimiterMath::clamp(in.vy, cfg.vy.min_value, cfg.vy.max_value);
    out.wz = CommandLimiterMath::clamp(in.wz, cfg.wz.min_value, cfg.wz.max_value);

    out.vx = CommandLimiterMath::apply_deadband(out.vx, cfg.vx.deadband);
    out.vy = CommandLimiterMath::apply_deadband(out.vy, cfg.vy.deadband);
    out.wz = CommandLimiterMath::apply_deadband(out.wz, cfg.wz.deadband);
    return out;
  }

private:
  CommandLimiterConfig config_ {};
  MotionCommand prev_output_ {};
  bool has_prev_output_ {false};
};

}  // namespace savo_control