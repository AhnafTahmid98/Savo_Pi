#pragma once

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>

#include "savo_control/command_limiter.hpp"
#include "savo_control/control_math.hpp"

namespace savo_control
{

enum class StuckState
{
  CLEAR,
  SUPPRESSED,
  STALE,
  COMMAND_ACTIVE,
  STUCK_DETECTED,
  RECOVERY_REQUESTED
};

inline std::string to_string(const StuckState state)
{
  switch (state) {
    case StuckState::CLEAR:
      return "CLEAR";
    case StuckState::SUPPRESSED:
      return "SUPPRESSED";
    case StuckState::STALE:
      return "STALE";
    case StuckState::COMMAND_ACTIVE:
      return "COMMAND_ACTIVE";
    case StuckState::STUCK_DETECTED:
      return "STUCK_DETECTED";
    case StuckState::RECOVERY_REQUESTED:
      return "RECOVERY_REQUESTED";
  }

  return "CLEAR";
}

inline std::string stuck_state_to_string(const StuckState state)
{
  return to_string(state);
}

inline bool is_stuck_state_active(const StuckState state)
{
  return state == StuckState::COMMAND_ACTIVE ||
         state == StuckState::STUCK_DETECTED ||
         state == StuckState::RECOVERY_REQUESTED;
}

inline bool is_stuck_detected_state(const StuckState state)
{
  return state == StuckState::STUCK_DETECTED ||
         state == StuckState::RECOVERY_REQUESTED;
}

inline bool can_request_recovery(const StuckState state)
{
  return state == StuckState::STUCK_DETECTED;
}

struct ObservedMotion
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};

  ObservedMotion sanitized() const
  {
    return ObservedMotion{
      ControlMath::finite_or_zero(vx),
      ControlMath::finite_or_zero(vy),
      ControlMath::finite_or_zero(wz)};
  }

  double linear_speed() const
  {
    const auto safe = sanitized();
    return ControlMath::hypot2(safe.vx, safe.vy);
  }
};

struct StuckDetectorConfig
{
  double command_linear_threshold{0.04};
  double command_angular_threshold{0.10};

  double observed_linear_threshold{0.015};
  double observed_angular_threshold{0.04};

  double stuck_duration_s{1.00};
  double stale_timeout_s{0.50};

  bool enabled{true};
  bool detect_vx{true};
  bool detect_vy{true};
  bool detect_wz{true};
  bool require_all_requested_axes_stopped{false};

  StuckDetectorConfig sanitized() const
  {
    StuckDetectorConfig out = *this;

    out.command_linear_threshold = std::abs(
      ControlMath::finite_or_zero(out.command_linear_threshold));
    out.command_angular_threshold = std::abs(
      ControlMath::finite_or_zero(out.command_angular_threshold));

    out.observed_linear_threshold = std::abs(
      ControlMath::finite_or_zero(out.observed_linear_threshold));
    out.observed_angular_threshold = std::abs(
      ControlMath::finite_or_zero(out.observed_angular_threshold));

    if (!std::isfinite(out.stuck_duration_s) || out.stuck_duration_s < 0.0) {
      out.stuck_duration_s = 1.00;
    }

    if (!std::isfinite(out.stale_timeout_s) || out.stale_timeout_s < 0.0) {
      out.stale_timeout_s = 0.50;
    }

    return out;
  }
};

struct StuckDetectorResult
{
  StuckState state{StuckState::CLEAR};

  bool stuck{false};
  bool recovery_requested{false};
  bool command_active{false};
  bool observed_moving{false};
  bool stale{false};
  bool enabled{true};

  double duration_s{0.0};
  double command_age_s{0.0};
  double odom_age_s{0.0};

  std::string reason{"clear"};
};

inline bool axis_requested(const double command_value, const double threshold)
{
  return std::abs(ControlMath::finite_or_zero(command_value)) >
         std::abs(ControlMath::finite_or_zero(threshold));
}

inline bool axis_observed(const double observed_value, const double threshold)
{
  return std::abs(ControlMath::finite_or_zero(observed_value)) >
         std::abs(ControlMath::finite_or_zero(threshold));
}

inline bool should_detect_stuck(
  const TwistCommand & cmd_vel_safe,
  const ObservedMotion & observed,
  const StuckDetectorConfig & config = StuckDetectorConfig{})
{
  const auto cfg = config.sanitized();
  if (!cfg.enabled) {
    return false;
  }

  const auto cmd = make_twist_command(
    cmd_vel_safe.vx,
    cmd_vel_safe.vy,
    cmd_vel_safe.wz);
  const auto obs = observed.sanitized();

  bool any_requested = false;
  bool any_requested_axis_stopped = false;
  bool all_requested_axes_stopped = true;

  if (cfg.detect_vx && axis_requested(cmd.vx, cfg.command_linear_threshold)) {
    any_requested = true;
    const bool stopped = !axis_observed(obs.vx, cfg.observed_linear_threshold);
    any_requested_axis_stopped = any_requested_axis_stopped || stopped;
    all_requested_axes_stopped = all_requested_axes_stopped && stopped;
  }

  if (cfg.detect_vy && axis_requested(cmd.vy, cfg.command_linear_threshold)) {
    any_requested = true;
    const bool stopped = !axis_observed(obs.vy, cfg.observed_linear_threshold);
    any_requested_axis_stopped = any_requested_axis_stopped || stopped;
    all_requested_axes_stopped = all_requested_axes_stopped && stopped;
  }

  if (cfg.detect_wz && axis_requested(cmd.wz, cfg.command_angular_threshold)) {
    any_requested = true;
    const bool stopped = !axis_observed(obs.wz, cfg.observed_angular_threshold);
    any_requested_axis_stopped = any_requested_axis_stopped || stopped;
    all_requested_axes_stopped = all_requested_axes_stopped && stopped;
  }

  if (!any_requested) {
    return false;
  }

  if (cfg.require_all_requested_axes_stopped) {
    return all_requested_axes_stopped;
  }

  return any_requested_axis_stopped;
}

class StuckDetector
{
public:
  StuckDetector()
  : config_(StuckDetectorConfig{}.sanitized())
  {
  }

  explicit StuckDetector(const StuckDetectorConfig & config)
  : config_(config.sanitized())
  {
  }

  void set_config(const StuckDetectorConfig & config)
  {
    config_ = config.sanitized();
  }

  const StuckDetectorConfig & config() const
  {
    return config_;
  }

  void reset()
  {
    state_ = StuckState::CLEAR;
    stuck_start_s_ = 0.0;
    has_stuck_start_ = false;
    recovery_requested_ = false;
  }

  void clear_recovery_request()
  {
    recovery_requested_ = false;
    if (state_ == StuckState::RECOVERY_REQUESTED) {
      state_ = StuckState::STUCK_DETECTED;
    }
  }

  void request_recovery()
  {
    if (state_ == StuckState::STUCK_DETECTED) {
      recovery_requested_ = true;
      state_ = StuckState::RECOVERY_REQUESTED;
    }
  }

  StuckDetectorResult update(
    const TwistCommand & cmd_vel_safe,
    const ObservedMotion & observed,
    const double now_s,
    const double last_command_s,
    const double last_odom_s,
    const bool suppress = false)
  {
    const double safe_now = ControlMath::finite_or_zero(now_s);
    const auto cfg = config_.sanitized();

    StuckDetectorResult result;
    result.enabled = cfg.enabled;

    if (!cfg.enabled) {
      reset();
      result.state = StuckState::SUPPRESSED;
      result.reason = "disabled";
      result.enabled = false;
      return result;
    }

    if (suppress) {
      reset();
      result.state = StuckState::SUPPRESSED;
      result.reason = "suppressed";
      return result;
    }

    result.command_age_s = std::max(0.0, safe_now - ControlMath::finite_or_zero(last_command_s));
    result.odom_age_s = std::max(0.0, safe_now - ControlMath::finite_or_zero(last_odom_s));

    if (result.command_age_s > cfg.stale_timeout_s || result.odom_age_s > cfg.stale_timeout_s) {
      reset();
      result.state = StuckState::STALE;
      result.stale = true;
      result.reason = "stale";
      state_ = result.state;
      return result;
    }

    const bool requested = command_requested(cmd_vel_safe, cfg);
    result.command_active = requested;

    if (!requested) {
      reset();
      result.state = StuckState::CLEAR;
      result.reason = "no_command";
      return result;
    }

    result.observed_moving = observed_moving_for_requested_axes(cmd_vel_safe, observed, cfg);

    const bool stuck_condition = should_detect_stuck(cmd_vel_safe, observed, cfg);

    if (!stuck_condition) {
      reset();
      result.state = StuckState::COMMAND_ACTIVE;
      result.reason = "motion_observed";
      state_ = result.state;
      return result;
    }

    if (!has_stuck_start_) {
      stuck_start_s_ = safe_now;
      has_stuck_start_ = true;
    }

    result.duration_s = std::max(0.0, safe_now - stuck_start_s_);

    if (result.duration_s >= cfg.stuck_duration_s) {
      result.stuck = true;
      result.recovery_requested = recovery_requested_;

      state_ = recovery_requested_
        ? StuckState::RECOVERY_REQUESTED
        : StuckState::STUCK_DETECTED;

      result.state = state_;
      result.reason = recovery_requested_ ? "recovery_requested" : "stuck_detected";
      return result;
    }

    state_ = StuckState::COMMAND_ACTIVE;
    result.state = state_;
    result.reason = "waiting_duration";
    return result;
  }

  StuckState state() const
  {
    return state_;
  }

  bool recovery_requested() const
  {
    return recovery_requested_;
  }

private:
  static bool command_requested(
    const TwistCommand & cmd,
    const StuckDetectorConfig & cfg)
  {
    return
      (cfg.detect_vx && axis_requested(cmd.vx, cfg.command_linear_threshold)) ||
      (cfg.detect_vy && axis_requested(cmd.vy, cfg.command_linear_threshold)) ||
      (cfg.detect_wz && axis_requested(cmd.wz, cfg.command_angular_threshold));
  }

  static bool observed_moving_for_requested_axes(
    const TwistCommand & cmd,
    const ObservedMotion & observed,
    const StuckDetectorConfig & cfg)
  {
    const auto obs = observed.sanitized();

    bool moving = false;

    if (cfg.detect_vx && axis_requested(cmd.vx, cfg.command_linear_threshold)) {
      moving = moving || axis_observed(obs.vx, cfg.observed_linear_threshold);
    }

    if (cfg.detect_vy && axis_requested(cmd.vy, cfg.command_linear_threshold)) {
      moving = moving || axis_observed(obs.vy, cfg.observed_linear_threshold);
    }

    if (cfg.detect_wz && axis_requested(cmd.wz, cfg.command_angular_threshold)) {
      moving = moving || axis_observed(obs.wz, cfg.observed_angular_threshold);
    }

    return moving;
  }

  StuckDetectorConfig config_{};
  StuckState state_{StuckState::CLEAR};

  double stuck_start_s_{0.0};
  bool has_stuck_start_{false};
  bool recovery_requested_{false};
};

inline std::ostream & operator<<(std::ostream & os, const StuckState state)
{
  os << to_string(state);
  return os;
}

}  // namespace savo_control
