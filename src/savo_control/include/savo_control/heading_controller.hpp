#pragma once

// =============================================================================
// Robot SAVO — savo_control / heading_controller.hpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Reusable heading/yaw controller utility for Robot Savo motion control.
//
// This class is built on top of the scalar PID controller (`pid.hpp`) and is
// intended for use in higher-level control nodes such as:
//   - heading_pid_node.cpp        (hold heading while moving)
//   - rotate_to_heading_node.cpp  (rotate in place to target heading)
//   - future follow / docking alignment controllers
//
// What this class provides
// ------------------------
// - Heading target management (set/clear/hold-current)
// - Wrapped angular error computation (shortest angular distance)
// - PID-based yaw-rate command generation (angular.z)
// - Optional output deadband / min/max yaw-rate shaping helpers
// - Goal tolerance checks for rotate-to-heading behavior
//
// Design note
// -----------
// This utility is ROS-independent. Nodes should:
//   - read yaw from odometry/IMU
//   - pass current yaw + dt into this class
//   - publish returned yaw-rate command to Twist.angular.z
//
// Mecanum suitability
// -------------------
// Fully suitable for mecanum robots. This class controls heading (yaw / wz)
// only. It does NOT perform wheel kinematics or motor mixing.
//
// Important note about update_to_target()
// ---------------------------------------
// The `update_to_target()` method below is implemented as a truly temporary
// target update path (no persistent target mutation, no PID reset side-effect
// from set_target_yaw()). This is intentional so it can be safely used for
// one-shot computations without altering the controller's stored target state.
// =============================================================================

#include <cmath>
#include <limits>

#include "savo_control/control_math.hpp"
#include "savo_control/pid.hpp"

namespace savo_control
{

// -----------------------------------------------------------------------------
// Config for heading controller behavior (around the underlying PID)
// -----------------------------------------------------------------------------
struct HeadingControllerConfig
{
  // Underlying yaw PID configuration (output typically maps to angular.z [rad/s])
  PidConfig pid {};

  // Heading goal tolerance for "at target" checks (radians)
  double heading_tolerance_rad {0.03};   // ~1.7 deg

  // Optional additional output deadband for final yaw-rate command.
  // Useful to avoid tiny command chatter near zero.
  double output_deadband_rad_s {0.0};

  // Optional minimum command magnitude when command is non-zero.
  // Helps overcome friction/stiction on some robots (careful when tuning).
  // If <= 0, disabled.
  double min_effective_wz_rad_s {0.0};

  // Optional max command magnitude clamp applied after PID output.
  // If <= 0, rely only on PID output clamp.
  double max_wz_rad_s {0.0};

  // If true, when entering hold mode, set target to current yaw and reset PID.
  bool reset_pid_on_hold_capture {true};

  // If true, reset PID whenever target heading changes significantly.
  bool reset_pid_on_target_change {true};

  // Threshold for "target changed significantly" reset behavior (radians)
  double target_change_reset_threshold_rad {0.02};

  // dt validity (controller-level guard; PID also has dt checks)
  double min_dt_sec {1e-6};
  double max_dt_sec {1.0};
};

// -----------------------------------------------------------------------------
// Result/debug output bundle
// -----------------------------------------------------------------------------
struct HeadingControllerResult
{
  // Final commanded yaw rate (to be used as Twist.angular.z)
  double wz_cmd_rad_s {0.0};

  // Target / current / error for diagnostics
  double target_yaw_rad {0.0};
  double current_yaw_rad {0.0};
  double error_yaw_rad {0.0};

  // Goal checks
  bool has_target {false};
  bool within_tolerance {false};

  // Update validity
  bool valid {false};
  bool dt_valid {false};
  bool yaw_valid {false};

  // PID breakdown for logging/tuning
  PidResult pid {};
};

// -----------------------------------------------------------------------------
// Heading/yaw controller
// -----------------------------------------------------------------------------
class HeadingController
{
public:
  HeadingController() = default;

  explicit HeadingController(const HeadingControllerConfig & cfg)
  : config_(cfg), pid_(cfg.pid)
  {
    normalize_config_();
    pid_.set_config(config_.pid);
  }

  // -------------------------
  // Configuration
  // -------------------------
  void set_config(const HeadingControllerConfig & cfg)
  {
    config_ = cfg;
    normalize_config_();
    pid_.set_config(config_.pid);
  }

  const HeadingControllerConfig & config() const
  {
    return config_;
  }

  Pid & pid()
  {
    return pid_;
  }

  const Pid & pid() const
  {
    return pid_;
  }

  // -------------------------
  // Target management
  // -------------------------
  bool has_target() const
  {
    return has_target_;
  }

  double target_yaw_rad() const
  {
    return target_yaw_rad_;
  }

  // Set absolute heading target (radians). Internally wrapped to [-pi, pi].
  void set_target_yaw(double target_yaw_rad)
  {
    if (!std::isfinite(target_yaw_rad)) {
      return;
    }

    const double new_target = ControlMath::wrap_angle_rad(target_yaw_rad);

    if (has_target_ && config_.reset_pid_on_target_change) {
      const double delta = std::abs(
        ControlMath::shortest_angular_distance_rad(target_yaw_rad_, new_target));
      if (delta >= std::abs(config_.target_change_reset_threshold_rad)) {
        pid_.reset();
      }
    }

    target_yaw_rad_ = new_target;
    has_target_ = true;
  }

  // Capture current yaw as target (heading hold behavior).
  void capture_hold_target(double current_yaw_rad)
  {
    if (!std::isfinite(current_yaw_rad)) {
      return;
    }

    target_yaw_rad_ = ControlMath::wrap_angle_rad(current_yaw_rad);
    has_target_ = true;

    if (config_.reset_pid_on_hold_capture) {
      pid_.reset();
    }
  }

  // Clear target and reset internal controller state
  void clear_target()
  {
    has_target_ = false;
    target_yaw_rad_ = 0.0;
    pid_.reset();
  }

  // -------------------------
  // State management
  // -------------------------
  void reset()
  {
    pid_.reset();
  }

  // -------------------------
  // Goal checks
  // -------------------------
  bool is_within_tolerance(double current_yaw_rad) const
  {
    if (!has_target_ || !std::isfinite(current_yaw_rad)) {
      return false;
    }

    const double err = ControlMath::shortest_angular_distance_rad(
      current_yaw_rad, target_yaw_rad_);

    return std::abs(err) <= std::abs(config_.heading_tolerance_rad);
  }

  // -------------------------
  // Main update (uses stored target)
  // -------------------------
  HeadingControllerResult update(double current_yaw_rad, double dt_sec)
  {
    HeadingControllerResult out{};
    out.current_yaw_rad = current_yaw_rad;
    out.dt_valid = valid_dt_(dt_sec);
    out.yaw_valid = std::isfinite(current_yaw_rad);
    out.has_target = has_target_;
    out.valid = false;

    if (!out.yaw_valid) {
      // Invalid yaw -> safe zero output
      out.wz_cmd_rad_s = 0.0;
      return out;
    }

    const double current_wrapped = ControlMath::wrap_angle_rad(current_yaw_rad);
    out.current_yaw_rad = current_wrapped;

    if (!has_target_) {
      // No target -> no yaw command
      out.wz_cmd_rad_s = 0.0;
      return out;
    }

    out.target_yaw_rad = target_yaw_rad_;

    // Error sign convention:
    // shortest_angular_distance(from=current, to=target)
    // Positive error => rotate CCW, Negative => rotate CW
    out.error_yaw_rad = ControlMath::shortest_angular_distance_rad(
      current_wrapped, target_yaw_rad_);

    out.within_tolerance = std::abs(out.error_yaw_rad) <=
                           std::abs(config_.heading_tolerance_rad);

    // Update PID on wrapped angular error
    out.pid = pid_.update(out.error_yaw_rad, dt_sec);

    // If PID update invalid, fail safe to zero
    if (!out.pid.valid) {
      out.wz_cmd_rad_s = 0.0;
      return out;
    }

    double wz_cmd = out.pid.output;

    // Optional external clamp (in addition to PID output clamp)
    if (config_.max_wz_rad_s > 0.0 && std::isfinite(config_.max_wz_rad_s)) {
      wz_cmd = ControlMath::saturate_abs(wz_cmd, std::abs(config_.max_wz_rad_s));
    }

    // Optional minimum effective command (anti-stiction helper)
    if (config_.min_effective_wz_rad_s > 0.0 && std::isfinite(config_.min_effective_wz_rad_s)) {
      const double min_eff = std::abs(config_.min_effective_wz_rad_s);
      if (std::abs(wz_cmd) > 0.0 && std::abs(wz_cmd) < min_eff) {
        wz_cmd = ControlMath::copy_sign(min_eff, wz_cmd);
      }
    }

    // Optional output deadband (final cleanup)
    wz_cmd = ControlMath::apply_deadband(wz_cmd, config_.output_deadband_rad_s);

    // If we are within tolerance, many rotate controllers prefer zero command.
    // Keep this behavior here as a safe default for reuse.
    if (out.within_tolerance) {
      wz_cmd = 0.0;
    }

    out.wz_cmd_rad_s = wz_cmd;
    out.valid = out.dt_valid && out.yaw_valid && out.pid.valid;
    return out;
  }

  // -------------------------
  // Convenience update with explicit temporary target
  // -------------------------
  // This method does NOT modify the stored target state and does NOT call
  // set_target_yaw(), specifically to avoid accidental PID reset side-effects
  // from reset_pid_on_target_change during one-shot computations.
  //
  // Use cases:
  // - temporary target evaluation in a test routine
  // - explicit rotate control path where persistent target storage is not needed
  //
  // If you want persistent target behavior, use:
  //   set_target_yaw(...)
  //   update(...)
  HeadingControllerResult update_to_target(
    double current_yaw_rad,
    double target_yaw_rad,
    double dt_sec)
  {
    HeadingControllerResult out{};
    out.current_yaw_rad = current_yaw_rad;
    out.dt_valid = valid_dt_(dt_sec);
    out.yaw_valid = std::isfinite(current_yaw_rad);
    out.has_target = std::isfinite(target_yaw_rad);
    out.valid = false;

    if (!out.yaw_valid || !std::isfinite(target_yaw_rad)) {
      out.wz_cmd_rad_s = 0.0;
      out.has_target = false;
      return out;
    }

    const double current_wrapped = ControlMath::wrap_angle_rad(current_yaw_rad);
    const double target_wrapped  = ControlMath::wrap_angle_rad(target_yaw_rad);

    out.current_yaw_rad = current_wrapped;
    out.target_yaw_rad = target_wrapped;
    out.error_yaw_rad = ControlMath::shortest_angular_distance_rad(
      current_wrapped, target_wrapped);

    out.within_tolerance = std::abs(out.error_yaw_rad) <=
                           std::abs(config_.heading_tolerance_rad);

    // Direct PID update on wrapped error (no stored target mutation)
    out.pid = pid_.update(out.error_yaw_rad, dt_sec);

    if (!out.pid.valid) {
      out.wz_cmd_rad_s = 0.0;
      return out;
    }

    double wz_cmd = out.pid.output;

    if (config_.max_wz_rad_s > 0.0 && std::isfinite(config_.max_wz_rad_s)) {
      wz_cmd = ControlMath::saturate_abs(wz_cmd, std::abs(config_.max_wz_rad_s));
    }

    if (config_.min_effective_wz_rad_s > 0.0 && std::isfinite(config_.min_effective_wz_rad_s)) {
      const double min_eff = std::abs(config_.min_effective_wz_rad_s);
      if (std::abs(wz_cmd) > 0.0 && std::abs(wz_cmd) < min_eff) {
        wz_cmd = ControlMath::copy_sign(min_eff, wz_cmd);
      }
    }

    wz_cmd = ControlMath::apply_deadband(wz_cmd, config_.output_deadband_rad_s);

    if (out.within_tolerance) {
      wz_cmd = 0.0;
    }

    out.wz_cmd_rad_s = wz_cmd;
    out.valid = out.dt_valid && out.yaw_valid && out.pid.valid;
    return out;
  }

private:
  bool valid_dt_(double dt_sec) const
  {
    return std::isfinite(dt_sec) &&
           dt_sec >= config_.min_dt_sec &&
           dt_sec <= config_.max_dt_sec;
  }

  void normalize_config_()
  {
    if (!std::isfinite(config_.heading_tolerance_rad) || config_.heading_tolerance_rad < 0.0) {
      config_.heading_tolerance_rad = 0.03;
    }

    if (!std::isfinite(config_.output_deadband_rad_s) || config_.output_deadband_rad_s < 0.0) {
      config_.output_deadband_rad_s = 0.0;
    }

    if (!std::isfinite(config_.min_effective_wz_rad_s) || config_.min_effective_wz_rad_s < 0.0) {
      config_.min_effective_wz_rad_s = 0.0;
    }

    if (!std::isfinite(config_.max_wz_rad_s)) {
      config_.max_wz_rad_s = 0.0;
    } else if (config_.max_wz_rad_s < 0.0) {
      config_.max_wz_rad_s = std::abs(config_.max_wz_rad_s);
    }

    if (!std::isfinite(config_.target_change_reset_threshold_rad) ||
        config_.target_change_reset_threshold_rad < 0.0)
    {
      config_.target_change_reset_threshold_rad = 0.02;
    }

    if (!std::isfinite(config_.min_dt_sec) || config_.min_dt_sec <= 0.0) {
      config_.min_dt_sec = 1e-6;
    }
    if (!std::isfinite(config_.max_dt_sec) || config_.max_dt_sec < config_.min_dt_sec) {
      config_.max_dt_sec = std::max(1.0, config_.min_dt_sec);
    }
  }

private:
  HeadingControllerConfig config_ {};
  Pid pid_ {};

  bool has_target_ {false};
  double target_yaw_rad_ {0.0};
};

}  // namespace savo_control