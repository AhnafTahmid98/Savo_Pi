#pragma once

#include <cmath>
#include <string>

#include "savo_control/control_math.hpp"
#include "savo_control/pid.hpp"

namespace savo_control
{

struct DistanceControllerConfig
{
  double target_distance_m{0.60};
  double tolerance_m{0.04};
  double hard_min_distance_m{0.35};

  double min_valid_distance_m{0.05};
  double max_valid_distance_m{3.00};
  double distance_timeout_s{0.40};

  double kp{0.45};
  double ki{0.0};
  double kd{0.03};

  double max_forward_vx{0.10};
  bool allow_reverse{false};
  double max_reverse_vx{0.05};

  double min_vx_when_active{0.04};
  double disable_min_vx_below_error_m{0.08};

  double output_deadband_m_s{0.0};

  double min_dt_sec{1.0e-4};
  double max_dt_sec{0.50};

  DistanceControllerConfig sanitized() const
  {
    DistanceControllerConfig out = *this;

    out.target_distance_m = std::max(0.0, ControlMath::finite_or_zero(out.target_distance_m));
    out.tolerance_m = std::max(0.0, ControlMath::finite_or_zero(out.tolerance_m));
    out.hard_min_distance_m = std::max(0.0, ControlMath::finite_or_zero(out.hard_min_distance_m));

    out.min_valid_distance_m = std::max(0.0, ControlMath::finite_or_zero(out.min_valid_distance_m));
    out.max_valid_distance_m = std::max(
      out.min_valid_distance_m,
      ControlMath::finite_or_zero(out.max_valid_distance_m));

    out.distance_timeout_s = (
      std::isfinite(out.distance_timeout_s) && out.distance_timeout_s > 0.0)
      ? out.distance_timeout_s
      : 0.40;

    out.kp = ControlMath::finite_or_zero(out.kp);
    out.ki = ControlMath::finite_or_zero(out.ki);
    out.kd = ControlMath::finite_or_zero(out.kd);

    out.max_forward_vx = std::abs(ControlMath::finite_or_zero(out.max_forward_vx));
    out.max_reverse_vx = std::abs(ControlMath::finite_or_zero(out.max_reverse_vx));

    out.min_vx_when_active = std::abs(ControlMath::finite_or_zero(out.min_vx_when_active));
    out.min_vx_when_active = std::min(out.min_vx_when_active, out.max_forward_vx);

    out.disable_min_vx_below_error_m = std::max(
      0.0,
      ControlMath::finite_or_zero(out.disable_min_vx_below_error_m));

    out.output_deadband_m_s = std::abs(ControlMath::finite_or_zero(out.output_deadband_m_s));

    if (!std::isfinite(out.min_dt_sec) || out.min_dt_sec <= 0.0) {
      out.min_dt_sec = 1.0e-4;
    }

    if (!std::isfinite(out.max_dt_sec) || out.max_dt_sec < out.min_dt_sec) {
      out.max_dt_sec = std::max(0.50, out.min_dt_sec);
    }

    return out;
  }

  PidConfig pid_config() const
  {
    const auto safe = sanitized();

    PidConfig cfg;
    cfg.kp = safe.kp;
    cfg.ki = safe.ki;
    cfg.kd = safe.kd;
    cfg.output_min = safe.allow_reverse ? -safe.max_reverse_vx : 0.0;
    cfg.output_max = safe.max_forward_vx;
    cfg.integral_clamp = safe.max_forward_vx > 0.0 ? safe.max_forward_vx : 1.0;
    cfg.min_dt_sec = safe.min_dt_sec;
    cfg.max_dt_sec = safe.max_dt_sec;
    return cfg.sanitized();
  }
};

struct DistanceControllerResult
{
  double vx_cmd_m_s{0.0};
  double output_raw_m_s{0.0};

  double distance_m{0.0};
  double target_distance_m{0.0};
  double error_m{0.0};

  PidResult pid{};

  bool valid{false};
  bool distance_valid{false};
  bool within_tolerance{false};
  bool hard_min_stop{false};
  bool reverse_blocked{false};
  bool direction_limited{false};
  bool deadband_applied{false};

  std::string reason{"invalid"};
};

class DistancePid
{
public:
  DistancePid()
  : config_(DistanceControllerConfig{}.sanitized()),
    pid_(config_.pid_config())
  {
  }

  explicit DistancePid(const DistanceControllerConfig & config)
  : config_(config.sanitized()),
    pid_(config_.pid_config())
  {
  }

  void set_config(const DistanceControllerConfig & config)
  {
    config_ = config.sanitized();
    pid_.set_config(config_.pid_config());
  }

  const DistanceControllerConfig & config() const
  {
    return config_;
  }

  void reset()
  {
    pid_.reset();
  }

  void set_target_distance(const double target_distance_m)
  {
    config_.target_distance_m = std::max(0.0, ControlMath::finite_or_zero(target_distance_m));
    config_ = config_.sanitized();
    pid_.set_config(config_.pid_config());
  }

  double target_distance() const
  {
    return config_.target_distance_m;
  }

  DistanceControllerResult update(const double distance_m, const double dt_sec)
  {
    config_ = config_.sanitized();

    DistanceControllerResult result;
    result.distance_m = ControlMath::finite_or_zero(distance_m);
    result.target_distance_m = config_.target_distance_m;
    result.error_m = result.distance_m - config_.target_distance_m;

    result.distance_valid = is_distance_valid(distance_m);
    if (!result.distance_valid) {
      result.valid = false;
      result.reason = "invalid_distance";
      result.vx_cmd_m_s = 0.0;
      result.output_raw_m_s = 0.0;
      return result;
    }

    result.valid = true;

    if (result.distance_m <= config_.hard_min_distance_m) {
      result.hard_min_stop = true;
      result.reason = "hard_min_distance";
      result.vx_cmd_m_s = 0.0;
      result.output_raw_m_s = 0.0;
      pid_.reset();
      return result;
    }

    if (std::abs(result.error_m) <= config_.tolerance_m) {
      result.within_tolerance = true;
      result.reason = "within_tolerance";
      result.vx_cmd_m_s = 0.0;
      result.output_raw_m_s = 0.0;
      return result;
    }

    result.pid = pid_.update(result.error_m, dt_sec);
    result.output_raw_m_s = result.pid.output_raw;

    double vx = result.pid.output;

    if (!config_.allow_reverse && result.error_m < 0.0) {
      vx = 0.0;
      result.reverse_blocked = true;
      result.direction_limited = true;
    }

    if (config_.allow_reverse) {
      vx = ControlMath::clamp(vx, -config_.max_reverse_vx, config_.max_forward_vx);
    } else {
      vx = ControlMath::clamp(vx, 0.0, config_.max_forward_vx);
    }

    const double abs_error = std::abs(result.error_m);
    if (
      vx > 0.0 &&
      config_.min_vx_when_active > 0.0 &&
      abs_error >= config_.disable_min_vx_below_error_m)
    {
      vx = std::max(vx, config_.min_vx_when_active);
      vx = std::min(vx, config_.max_forward_vx);
    }

    if (config_.allow_reverse && vx < 0.0) {
      vx = std::max(vx, -config_.max_reverse_vx);
    }

    const double before_deadband = vx;
    vx = ControlMath::apply_deadband(vx, config_.output_deadband_m_s);
    result.deadband_applied = !ControlMath::nearly_equal(before_deadband, vx, 1.0e-12);

    result.vx_cmd_m_s = vx;

    if (result.reverse_blocked) {
      result.reason = "reverse_blocked";
    } else if (result.direction_limited) {
      result.reason = "direction_limited";
    } else {
      result.reason = "tracking";
    }

    return result;
  }

  bool is_distance_valid(const double distance_m) const
  {
    return std::isfinite(distance_m) &&
           distance_m >= config_.min_valid_distance_m &&
           distance_m <= config_.max_valid_distance_m;
  }

private:
  DistanceControllerConfig config_{};
  Pid pid_{};
};

using DistanceController = DistancePid;
using DistancePidConfig = DistanceControllerConfig;
using DistanceControllerConfigCpp = DistanceControllerConfig;
using DistancePidResult = DistanceControllerResult;
using DistanceControllerResultCpp = DistanceControllerResult;

}  // namespace savo_control
