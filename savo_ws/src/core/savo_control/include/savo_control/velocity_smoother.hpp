#pragma once

#include <algorithm>
#include <cmath>

#include "savo_control/command_limiter.hpp"
#include "savo_control/control_math.hpp"

namespace savo_control
{

struct VelocitySmootherConfig
{
  double max_accel_vx{0.40};
  double max_accel_vy{0.40};
  double max_accel_wz{1.20};

  double max_decel_vx{0.60};
  double max_decel_vy{0.60};
  double max_decel_wz{1.80};

  double command_timeout_s{0.50};
  double min_dt_s{1.0e-4};
  double max_dt_s{0.20};

  CommandLimits output_limits{};

  bool reset_on_timeout{true};
  bool use_decel_when_slowing{true};

  VelocitySmootherConfig sanitized() const
  {
    VelocitySmootherConfig out = *this;

    out.max_accel_vx = std::abs(ControlMath::finite_or_zero(out.max_accel_vx));
    out.max_accel_vy = std::abs(ControlMath::finite_or_zero(out.max_accel_vy));
    out.max_accel_wz = std::abs(ControlMath::finite_or_zero(out.max_accel_wz));

    out.max_decel_vx = std::abs(ControlMath::finite_or_zero(out.max_decel_vx));
    out.max_decel_vy = std::abs(ControlMath::finite_or_zero(out.max_decel_vy));
    out.max_decel_wz = std::abs(ControlMath::finite_or_zero(out.max_decel_wz));

    if (!std::isfinite(out.command_timeout_s) || out.command_timeout_s < 0.0) {
      out.command_timeout_s = 0.50;
    }

    if (!std::isfinite(out.min_dt_s) || out.min_dt_s <= 0.0) {
      out.min_dt_s = 1.0e-4;
    }

    if (!std::isfinite(out.max_dt_s) || out.max_dt_s < out.min_dt_s) {
      out.max_dt_s = std::max(0.20, out.min_dt_s);
    }

    out.output_limits = out.output_limits.sanitized();

    return out;
  }
};

struct VelocitySmootherState
{
  TwistCommand output{};
  TwistCommand target{};

  double last_update_s{0.0};
  double last_command_s{0.0};

  bool has_output{false};
  bool has_command{false};
  bool timed_out{false};
};

class VelocitySmoother
{
public:
  VelocitySmoother()
  : config_(VelocitySmootherConfig{}.sanitized()),
    limiter_(config_.output_limits)
  {
  }

  explicit VelocitySmoother(const VelocitySmootherConfig & config)
  : config_(config.sanitized()),
    limiter_(config_.output_limits)
  {
  }

  void set_config(const VelocitySmootherConfig & config)
  {
    config_ = config.sanitized();
    limiter_.set_limits(config_.output_limits);
  }

  const VelocitySmootherConfig & config() const
  {
    return config_;
  }

  const VelocitySmootherState & state() const
  {
    return state_;
  }

  void reset()
  {
    state_ = VelocitySmootherState{};
  }

  void reset_to_zero(const double now_s = 0.0)
  {
    state_.output = TwistCommand{};
    state_.target = TwistCommand{};
    state_.last_update_s = ControlMath::finite_or_zero(now_s);
    state_.last_command_s = ControlMath::finite_or_zero(now_s);
    state_.has_output = true;
    state_.has_command = false;
    state_.timed_out = false;
  }

  void set_target(const TwistCommand & target, const double now_s)
  {
    state_.target = limiter_.limit(target);
    state_.last_command_s = ControlMath::finite_or_zero(now_s);
    state_.has_command = true;
    state_.timed_out = false;
  }

  TwistCommand target() const
  {
    return state_.target;
  }

  TwistCommand output() const
  {
    return state_.output;
  }

  TwistCommand update(const TwistCommand & target, const double now_s)
  {
    set_target(target, now_s);
    return update(now_s);
  }

  TwistCommand update(const double now_s)
  {
    const double safe_now = ControlMath::finite_or_zero(now_s);

    if (!state_.has_output) {
      state_.output = TwistCommand{};
      state_.last_update_s = safe_now;
      state_.has_output = true;
    }

    if (is_timed_out(safe_now)) {
      state_.timed_out = true;

      if (config_.reset_on_timeout) {
        state_.target = TwistCommand{};
      }
    }

    double dt_s = safe_now - state_.last_update_s;
    if (!std::isfinite(dt_s) || dt_s < config_.min_dt_s) {
      dt_s = config_.min_dt_s;
    }
    if (dt_s > config_.max_dt_s) {
      dt_s = config_.max_dt_s;
    }

    state_.output.vx = smooth_axis(
      state_.output.vx,
      state_.target.vx,
      config_.max_accel_vx,
      config_.max_decel_vx,
      dt_s);

    state_.output.vy = smooth_axis(
      state_.output.vy,
      state_.target.vy,
      config_.max_accel_vy,
      config_.max_decel_vy,
      dt_s);

    state_.output.wz = smooth_axis(
      state_.output.wz,
      state_.target.wz,
      config_.max_accel_wz,
      config_.max_decel_wz,
      dt_s);

    state_.output = limiter_.limit(state_.output);
    state_.last_update_s = safe_now;

    return state_.output;
  }

  TwistCommand stop(const double now_s)
  {
    set_target(TwistCommand{}, now_s);
    return update(now_s);
  }

  bool is_timed_out(const double now_s) const
  {
    if (!state_.has_command) {
      return false;
    }

    if (config_.command_timeout_s <= 0.0) {
      return false;
    }

    const double age_s = ControlMath::finite_or_zero(now_s) - state_.last_command_s;
    return age_s > config_.command_timeout_s;
  }

  bool has_output() const
  {
    return state_.has_output;
  }

  bool has_command() const
  {
    return state_.has_command;
  }

  bool timed_out() const
  {
    return state_.timed_out;
  }

private:
  double smooth_axis(
    const double current,
    const double target,
    const double accel_limit,
    const double decel_limit,
    const double dt_s) const
  {
    const double safe_current = ControlMath::finite_or_zero(current);
    const double safe_target = ControlMath::finite_or_zero(target);

    const double delta = safe_target - safe_current;

    double rate_limit = std::abs(accel_limit);
    if (config_.use_decel_when_slowing && is_slowing_down(safe_current, safe_target)) {
      rate_limit = std::abs(decel_limit);
    }

    if (!std::isfinite(rate_limit) || rate_limit <= 0.0) {
      return safe_target;
    }

    const double max_step = rate_limit * std::max(0.0, dt_s);
    return ControlMath::step_toward(safe_current, safe_target, max_step);
  }

  static bool is_slowing_down(const double current, const double target)
  {
    if (ControlMath::near_zero(current)) {
      return false;
    }

    if (current > 0.0) {
      return target < current;
    }

    return target > current;
  }

  VelocitySmootherConfig config_{};
  VelocitySmootherState state_{};
  CommandLimiter limiter_{};
};

}  // namespace savo_control
