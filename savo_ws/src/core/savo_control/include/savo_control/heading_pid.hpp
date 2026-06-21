#pragma once

#include <cmath>
#include <string>

#include "savo_control/control_math.hpp"
#include "savo_control/pid.hpp"

namespace savo_control
{

inline double wrap_heading_rad(const double angle_rad)
{
  return ControlMath::wrap_angle_rad(angle_rad);
}

inline double heading_shortest_angular_distance_rad(
  const double from_rad,
  const double to_rad)
{
  return ControlMath::shortest_angular_distance_rad(from_rad, to_rad);
}

inline double saturate_abs(const double value, const double abs_limit)
{
  return ControlMath::saturate_abs(value, abs_limit);
}

inline double copy_sign(const double magnitude, const double sign_source)
{
  return ControlMath::copy_sign(magnitude, sign_source);
}

struct HeadingControllerConfig
{
  double target_yaw_rad{0.0};
  double tolerance_rad{0.035};

  double kp{1.20};
  double ki{0.0};
  double kd{0.05};

  double max_wz_rad_s{0.45};
  double min_wz_when_active{0.08};
  double disable_min_wz_below_error_rad{0.08};

  double output_deadband_rad_s{0.0};

  double min_dt_sec{1.0e-4};
  double max_dt_sec{0.50};

  HeadingControllerConfig sanitized() const
  {
    HeadingControllerConfig out = *this;

    out.target_yaw_rad = ControlMath::wrap_angle_rad(
      ControlMath::finite_or_zero(out.target_yaw_rad));
    out.tolerance_rad = std::max(0.0, ControlMath::finite_or_zero(out.tolerance_rad));

    out.kp = ControlMath::finite_or_zero(out.kp);
    out.ki = ControlMath::finite_or_zero(out.ki);
    out.kd = ControlMath::finite_or_zero(out.kd);

    out.max_wz_rad_s = std::abs(ControlMath::finite_or_zero(out.max_wz_rad_s));
    out.min_wz_when_active = std::abs(ControlMath::finite_or_zero(out.min_wz_when_active));
    out.min_wz_when_active = std::min(out.min_wz_when_active, out.max_wz_rad_s);

    out.disable_min_wz_below_error_rad = std::max(
      0.0,
      ControlMath::finite_or_zero(out.disable_min_wz_below_error_rad));

    out.output_deadband_rad_s = std::abs(
      ControlMath::finite_or_zero(out.output_deadband_rad_s));

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
    cfg.output_min = -safe.max_wz_rad_s;
    cfg.output_max = safe.max_wz_rad_s;
    cfg.integral_clamp = safe.max_wz_rad_s > 0.0 ? safe.max_wz_rad_s : 1.0;
    cfg.min_dt_sec = safe.min_dt_sec;
    cfg.max_dt_sec = safe.max_dt_sec;
    return cfg.sanitized();
  }
};

struct HeadingControllerResult
{
  double wz_cmd_rad_s{0.0};
  double output_raw_rad_s{0.0};

  double current_yaw_rad{0.0};
  double target_yaw_rad{0.0};
  double error_rad{0.0};

  PidResult pid{};

  bool valid{false};
  bool yaw_valid{false};
  bool within_tolerance{false};
  bool min_output_applied{false};
  bool deadband_applied{false};
  bool saturated{false};

  std::string reason{"invalid"};
};

class HeadingPid
{
public:
  HeadingPid()
  : config_(HeadingControllerConfig{}.sanitized()),
    pid_(config_.pid_config())
  {
  }

  explicit HeadingPid(const HeadingControllerConfig & config)
  : config_(config.sanitized()),
    pid_(config_.pid_config())
  {
  }

  void set_config(const HeadingControllerConfig & config)
  {
    config_ = config.sanitized();
    pid_.set_config(config_.pid_config());
  }

  const HeadingControllerConfig & config() const
  {
    return config_;
  }

  void reset()
  {
    pid_.reset();
  }

  void set_target_yaw(const double target_yaw_rad)
  {
    config_.target_yaw_rad = ControlMath::wrap_angle_rad(
      ControlMath::finite_or_zero(target_yaw_rad));
    config_ = config_.sanitized();
    pid_.set_config(config_.pid_config());
  }

  void set_target_heading(const double target_yaw_rad)
  {
    set_target_yaw(target_yaw_rad);
  }

  double target_yaw() const
  {
    return config_.target_yaw_rad;
  }

  double target_heading() const
  {
    return target_yaw();
  }

  HeadingControllerResult update(const double current_yaw_rad, const double dt_sec)
  {
    return update(current_yaw_rad, config_.target_yaw_rad, dt_sec);
  }

  HeadingControllerResult update(
    const double current_yaw_rad,
    const double target_yaw_rad,
    const double dt_sec)
  {
    config_ = config_.sanitized();

    HeadingControllerResult result;
    result.current_yaw_rad = ControlMath::wrap_angle_rad(
      ControlMath::finite_or_zero(current_yaw_rad));
    result.target_yaw_rad = ControlMath::wrap_angle_rad(
      ControlMath::finite_or_zero(target_yaw_rad));

    result.yaw_valid = std::isfinite(current_yaw_rad) && std::isfinite(target_yaw_rad);
    if (!result.yaw_valid) {
      result.valid = false;
      result.reason = "invalid_yaw";
      result.wz_cmd_rad_s = 0.0;
      result.output_raw_rad_s = 0.0;
      pid_.reset();
      return result;
    }

    result.error_rad = ControlMath::shortest_angular_distance_rad(
      result.current_yaw_rad,
      result.target_yaw_rad);

    result.valid = true;

    if (std::abs(result.error_rad) <= config_.tolerance_rad) {
      result.within_tolerance = true;
      result.reason = "within_tolerance";
      result.wz_cmd_rad_s = 0.0;
      result.output_raw_rad_s = 0.0;
      return result;
    }

    result.pid = pid_.update(result.error_rad, dt_sec);
    result.output_raw_rad_s = result.pid.output_raw;

    double wz = result.pid.output;
    result.saturated = result.pid.saturated;

    const double abs_error = std::abs(result.error_rad);
    if (
      !ControlMath::near_zero(wz) &&
      config_.min_wz_when_active > 0.0 &&
      abs_error >= config_.disable_min_wz_below_error_rad &&
      std::abs(wz) < config_.min_wz_when_active)
    {
      wz = ControlMath::copy_sign(config_.min_wz_when_active, wz);
      result.min_output_applied = true;
    }

    wz = ControlMath::saturate_abs(wz, config_.max_wz_rad_s);

    const double before_deadband = wz;
    wz = ControlMath::apply_deadband(wz, config_.output_deadband_rad_s);
    result.deadband_applied = !ControlMath::nearly_equal(before_deadband, wz, 1.0e-12);

    result.wz_cmd_rad_s = wz;
    result.reason = "tracking";

    return result;
  }

  bool is_yaw_valid(const double yaw_rad) const
  {
    return std::isfinite(yaw_rad);
  }

private:
  HeadingControllerConfig config_{};
  Pid pid_{};
};

using HeadingController = HeadingPid;
using HeadingPidConfig = HeadingControllerConfig;
using HeadingControllerConfigCpp = HeadingControllerConfig;
using HeadingPidResult = HeadingControllerResult;
using HeadingControllerResultCpp = HeadingControllerResult;

}  // namespace savo_control
