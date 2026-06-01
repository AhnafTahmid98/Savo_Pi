#pragma once

// =============================================================================
// Robot SAVO — savo_control / pid.hpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Reusable PID controller utility for Robot Savo control nodes.
//
// This class is ROS-independent and intended for use in:
//   - heading_pid_node
//   - rotate_to_heading_node
//   - distance/approach controllers
//   - recovery alignment primitives (future)
//
// Features
// --------
// - P / I / D control with runtime gain updates
// - dt validation and safe fallback behavior
// - output saturation (min/max)
// - integral clamp (anti-windup, clamp-based)
// - optional derivative low-pass filtering
// - reset() / reset_to() for mode changes, stop events, and recovery transitions
//
// Design note
// -----------
// This PID class operates on scalar values only. Nodes are responsible for:
//   - computing the correct error signal (e.g., wrapped angular error)
//   - choosing dt from the control loop timing
//   - publishing outputs / handling ROS topics
// =============================================================================

#include <algorithm>
#include <cmath>
#include <limits>

namespace savo_control
{

// -----------------------------------------------------------------------------
// PID parameter set
// -----------------------------------------------------------------------------
struct PidConfig
{
  // Gains
  double kp {0.0};
  double ki {0.0};
  double kd {0.0};

  // Output clamp
  double output_min {-std::numeric_limits<double>::infinity()};
  double output_max { std::numeric_limits<double>::infinity()};

  // Integral term clamp (applied to accumulated integral state, not output)
  // If integral_clamp < 0, absolute value is used.
  double integral_clamp {std::numeric_limits<double>::infinity()};

  // Derivative low-pass filter coefficient in [0, 1].
  // 0.0 -> no filtering (raw derivative)
  // 1.0 -> very heavy filtering (slow derivative response)
  //
  // Typical starting value: 0.1 to 0.3 (if needed)
  double d_filter_alpha {0.0};

  // dt validity bounds to protect against bad timing spikes
  double min_dt_sec {1e-6};
  double max_dt_sec {1.0};

  // If true, integral accumulation pauses when error is non-finite or dt invalid
  // (recommended default behavior)
  bool freeze_integral_on_invalid_dt {true};

  // If true, derivative is computed on measurement change (caller must supply
  // measurement path separately; current class computes derivative on error only).
  // Reserved for future compatibility / clarity in config files.
  bool derivative_on_measurement {false};

  // Helper utilities
  void set_output_symmetric(double abs_limit)
  {
    const double a = std::abs(abs_limit);
    output_min = -a;
    output_max =  a;
  }

  void set_integral_clamp_symmetric(double abs_limit)
  {
    integral_clamp = std::abs(abs_limit);
  }
};

// -----------------------------------------------------------------------------
// PID debug/result output bundle
// -----------------------------------------------------------------------------
struct PidResult
{
  // Final saturated controller output
  double output {0.0};

  // Unsaturated raw sum (P+I+D before output clamp)
  double output_raw {0.0};

  // Term contributions
  double p_term {0.0};
  double i_term {0.0};
  double d_term {0.0};

  // Internal states (exposed for diagnostics)
  double error {0.0};
  double integral_state {0.0};
  double derivative_state {0.0};   // filtered derivative used in d_term
  double dt_sec {0.0};

  // Flags
  bool valid {false};
  bool dt_valid {false};
  bool saturated {false};
};

// -----------------------------------------------------------------------------
// Scalar PID controller
// -----------------------------------------------------------------------------
class Pid
{
public:
  Pid() = default;

  explicit Pid(const PidConfig & config)
  : config_(config)
  {}

  // -------------------------
  // Configuration
  // -------------------------
  void set_config(const PidConfig & config)
  {
    config_ = config;
    normalize_config_();
  }

  const PidConfig & config() const
  {
    return config_;
  }

  void set_gains(double kp, double ki, double kd)
  {
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
  }

  // -------------------------
  // State management
  // -------------------------
  void reset()
  {
    integral_state_ = 0.0;
    prev_error_ = 0.0;
    derivative_state_ = 0.0;
    has_prev_error_ = false;
    last_output_ = 0.0;
    has_last_output_ = false;
  }

  // Reset to a known state (useful after mode switch / recovery transition)
  void reset_to(double error, double output = 0.0)
  {
    integral_state_ = 0.0;
    prev_error_ = std::isfinite(error) ? error : 0.0;
    derivative_state_ = 0.0;
    has_prev_error_ = std::isfinite(error);
    last_output_ = std::isfinite(output) ? output : 0.0;
    has_last_output_ = std::isfinite(output);
  }

  bool has_previous_error() const
  {
    return has_prev_error_;
  }

  double previous_error() const
  {
    return prev_error_;
  }

  double integral_state() const
  {
    return integral_state_;
  }

  double derivative_state() const
  {
    return derivative_state_;
  }

  bool has_last_output() const
  {
    return has_last_output_;
  }

  double last_output() const
  {
    return last_output_;
  }

  // -------------------------
  // Compute PID output from error
  // -------------------------
  // Caller is responsible for:
  // - computing proper error signal (e.g., wrapped angle error)
  // - passing a valid dt
  PidResult update(double error, double dt_sec)
  {
    normalize_config_();

    PidResult r{};
    r.error = error;
    r.dt_sec = dt_sec;

    const bool error_ok = std::isfinite(error);
    const bool dt_ok = valid_dt_(dt_sec);
    r.dt_valid = dt_ok;

    if (!error_ok) {
      // Invalid input error -> return safe zero output and mark invalid
      r.output = 0.0;
      r.output_raw = 0.0;
      r.valid = false;
      last_output_ = r.output;
      has_last_output_ = true;
      return r;
    }

    // P term (always safe if error is finite)
    r.p_term = config_.kp * error;

    // Integral term
    if (dt_ok) {
      // Clamp-based anti-windup on integral state
      integral_state_ += error * dt_sec;

      const double i_clamp = std::abs(config_.integral_clamp);
      if (std::isfinite(i_clamp)) {
        integral_state_ = clamp_(integral_state_, -i_clamp, i_clamp);
      }
    } else if (config_.freeze_integral_on_invalid_dt) {
      // Hold integral as-is
    }

    r.integral_state = integral_state_;
    r.i_term = config_.ki * integral_state_;

    // Derivative term (on error)
    double d_raw = 0.0;
    if (dt_ok && has_prev_error_) {
      d_raw = (error - prev_error_) / dt_sec;
    } else {
      d_raw = 0.0;
    }

    // Optional derivative low-pass filtering
    const double alpha = clamp_(config_.d_filter_alpha, 0.0, 1.0);
    if (!has_prev_error_ || !dt_ok) {
      // Reset derivative state on first run / invalid dt to avoid spikes
      derivative_state_ = 0.0;
    } else if (alpha <= 0.0) {
      derivative_state_ = d_raw;
    } else {
      // Exponential smoothing:
      // new = (1-alpha)*raw + alpha*prev
      derivative_state_ = (1.0 - alpha) * d_raw + alpha * derivative_state_;
    }

    r.derivative_state = derivative_state_;
    r.d_term = config_.kd * derivative_state_;

    // Raw sum
    r.output_raw = r.p_term + r.i_term + r.d_term;

    // Output clamp
    const double out_min = std::min(config_.output_min, config_.output_max);
    const double out_max = std::max(config_.output_min, config_.output_max);
    r.output = clamp_(r.output_raw, out_min, out_max);
    r.saturated = !nearly_equal_(r.output, r.output_raw);

    // Update states for next iteration
    prev_error_ = error;
    has_prev_error_ = true;

    last_output_ = r.output;
    has_last_output_ = true;

    r.valid = true;
    return r;
  }

  // Convenience method:
  // updates from setpoint and measurement by computing error = setpoint - measurement.
  // (For angular control, prefer caller to pre-wrap error and use update(error, dt).)
  PidResult update_from_setpoint(double setpoint, double measurement, double dt_sec)
  {
    const double error = setpoint - measurement;
    return update(error, dt_sec);
  }

private:
  // -------------------------
  // Internal helpers
  // -------------------------
  static inline double clamp_(double value, double min_value, double max_value)
  {
    if (min_value > max_value) {
      std::swap(min_value, max_value);
    }
    return std::clamp(value, min_value, max_value);
  }

  static inline bool nearly_equal_(double a, double b, double eps = 1e-12)
  {
    return std::abs(a - b) <= eps;
  }

  bool valid_dt_(double dt_sec) const
  {
    return std::isfinite(dt_sec) &&
           dt_sec >= config_.min_dt_sec &&
           dt_sec <= config_.max_dt_sec;
  }

  void normalize_config_()
  {
    if (!std::isfinite(config_.kp)) { config_.kp = 0.0; }
    if (!std::isfinite(config_.ki)) { config_.ki = 0.0; }
    if (!std::isfinite(config_.kd)) { config_.kd = 0.0; }

    if (!std::isfinite(config_.output_min)) {
      config_.output_min = -std::numeric_limits<double>::infinity();
    }
    if (!std::isfinite(config_.output_max)) {
      config_.output_max =  std::numeric_limits<double>::infinity();
    }

    if (config_.output_min > config_.output_max) {
      std::swap(config_.output_min, config_.output_max);
    }

    if (!std::isfinite(config_.integral_clamp)) {
      config_.integral_clamp = std::numeric_limits<double>::infinity();
    } else {
      config_.integral_clamp = std::abs(config_.integral_clamp);
    }

    if (!std::isfinite(config_.d_filter_alpha)) {
      config_.d_filter_alpha = 0.0;
    }
    config_.d_filter_alpha = clamp_(config_.d_filter_alpha, 0.0, 1.0);

    if (!std::isfinite(config_.min_dt_sec) || config_.min_dt_sec <= 0.0) {
      config_.min_dt_sec = 1e-6;
    }
    if (!std::isfinite(config_.max_dt_sec) || config_.max_dt_sec < config_.min_dt_sec) {
      config_.max_dt_sec = std::max(1.0, config_.min_dt_sec);
    }
  }

private:
  PidConfig config_ {};

  // Internal state
  double integral_state_ {0.0};
  double prev_error_ {0.0};
  double derivative_state_ {0.0};

  bool has_prev_error_ {false};

  double last_output_ {0.0};
  bool has_last_output_ {false};
};

}  // namespace savo_control