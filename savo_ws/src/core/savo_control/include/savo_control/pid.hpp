#pragma once

#include <cmath>
#include <limits>

#include "savo_control/control_math.hpp"

namespace savo_control
{

struct PidConfig
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};

  double output_min{-std::numeric_limits<double>::infinity()};
  double output_max{std::numeric_limits<double>::infinity()};
  double integral_clamp{std::numeric_limits<double>::infinity()};

  double d_filter_alpha{0.0};

  double min_dt_sec{1.0e-6};
  double max_dt_sec{1.0};

  bool freeze_integral_on_invalid_dt{true};
  bool derivative_on_measurement{false};

  void set_output_symmetric(const double abs_limit)
  {
    const double limit = std::isfinite(abs_limit) ?
      std::abs(abs_limit) :
      std::numeric_limits<double>::infinity();

    output_min = -limit;
    output_max = limit;
  }

  void set_integral_clamp_symmetric(const double abs_limit)
  {
    integral_clamp = std::isfinite(abs_limit) ?
      std::abs(abs_limit) :
      std::numeric_limits<double>::infinity();
  }

  PidConfig sanitized() const
  {
    PidConfig out = *this;

    out.kp = ControlMath::finite_or_zero(out.kp);
    out.ki = ControlMath::finite_or_zero(out.ki);
    out.kd = ControlMath::finite_or_zero(out.kd);

    if (out.output_min > out.output_max) {
      const double tmp = out.output_min;
      out.output_min = out.output_max;
      out.output_max = tmp;
    }

    out.integral_clamp = std::isfinite(out.integral_clamp) ?
      std::abs(out.integral_clamp) :
      std::numeric_limits<double>::infinity();

    out.d_filter_alpha = std::isfinite(out.d_filter_alpha) ?
      ControlMath::clamp(out.d_filter_alpha, 0.0, 1.0) :
      0.0;

    if (!std::isfinite(out.min_dt_sec) || out.min_dt_sec <= 0.0) {
      out.min_dt_sec = 1.0e-6;
    }

    if (!std::isfinite(out.max_dt_sec) || out.max_dt_sec < out.min_dt_sec) {
      out.max_dt_sec = std::max(1.0, out.min_dt_sec);
    }

    return out;
  }
};

struct PidResult
{
  double output{0.0};
  double output_raw{0.0};

  double p_term{0.0};
  double i_term{0.0};
  double d_term{0.0};

  double error{0.0};
  double integral_state{0.0};
  double derivative_state{0.0};
  double dt_sec{0.0};

  bool valid{false};
  bool dt_valid{false};
  bool saturated{false};
};

class Pid
{
public:
  Pid()
  : config_(PidConfig{}.sanitized())
  {
  }

  explicit Pid(const PidConfig & config)
  : config_(config.sanitized())
  {
  }

  void set_config(const PidConfig & config)
  {
    config_ = config.sanitized();
  }

  const PidConfig & config() const
  {
    return config_;
  }

  void set_gains(const double kp, const double ki, const double kd)
  {
    config_.kp = ControlMath::finite_or_zero(kp);
    config_.ki = ControlMath::finite_or_zero(ki);
    config_.kd = ControlMath::finite_or_zero(kd);
    config_ = config_.sanitized();
  }

  void reset()
  {
    integral_state_ = 0.0;
    previous_error_ = 0.0;
    derivative_state_ = 0.0;
    has_previous_error_ = false;

    last_output_ = 0.0;
    has_last_output_ = false;
  }

  void reset_to(const double error, const double output = 0.0)
  {
    integral_state_ = 0.0;
    previous_error_ = std::isfinite(error) ? error : 0.0;
    derivative_state_ = 0.0;
    has_previous_error_ = std::isfinite(error);

    last_output_ = std::isfinite(output) ? output : 0.0;
    has_last_output_ = std::isfinite(output);
  }

  bool has_previous_error() const
  {
    return has_previous_error_;
  }

  double previous_error() const
  {
    return previous_error_;
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

  PidResult update(const double error, const double dt_sec)
  {
    config_ = config_.sanitized();

    PidResult result;
    result.error = error;
    result.dt_sec = dt_sec;

    const bool error_valid = std::isfinite(error);
    result.dt_valid = valid_dt(dt_sec);

    if (!error_valid) {
      last_output_ = 0.0;
      has_last_output_ = true;
      return result;
    }

    result.p_term = config_.kp * error;

    if (result.dt_valid) {
      integral_state_ += error * dt_sec;

      if (std::isfinite(config_.integral_clamp)) {
        const double limit = std::abs(config_.integral_clamp);
        integral_state_ = ControlMath::clamp(integral_state_, -limit, limit);
      }
    }

    result.integral_state = integral_state_;
    result.i_term = config_.ki * integral_state_;

    double derivative_raw = 0.0;
    if (result.dt_valid && has_previous_error_) {
      derivative_raw = (error - previous_error_) / dt_sec;
    }

    const double alpha = ControlMath::clamp(config_.d_filter_alpha, 0.0, 1.0);

    if (!has_previous_error_ || !result.dt_valid) {
      derivative_state_ = 0.0;
    } else if (alpha <= 0.0) {
      derivative_state_ = derivative_raw;
    } else {
      derivative_state_ = ((1.0 - alpha) * derivative_raw) + (alpha * derivative_state_);
    }

    result.derivative_state = derivative_state_;
    result.d_term = config_.kd * derivative_state_;

    result.output_raw = result.p_term + result.i_term + result.d_term;

    const double output_min = std::min(config_.output_min, config_.output_max);
    const double output_max = std::max(config_.output_min, config_.output_max);

    result.output = ControlMath::clamp(result.output_raw, output_min, output_max);
    result.saturated = !ControlMath::nearly_equal(result.output, result.output_raw, 1.0e-12);

    previous_error_ = error;
    has_previous_error_ = true;

    last_output_ = result.output;
    has_last_output_ = true;

    result.valid = true;
    return result;
  }

  PidResult update_from_setpoint(
    const double setpoint,
    const double measurement,
    const double dt_sec)
  {
    if (!std::isfinite(setpoint) || !std::isfinite(measurement)) {
      return update(std::numeric_limits<double>::quiet_NaN(), dt_sec);
    }

    return update(setpoint - measurement, dt_sec);
  }

private:
  bool valid_dt(const double dt_sec) const
  {
    return ControlMath::valid_dt(dt_sec, config_.min_dt_sec, config_.max_dt_sec);
  }

  PidConfig config_{};

  double integral_state_{0.0};
  double previous_error_{0.0};
  double derivative_state_{0.0};
  bool has_previous_error_{false};

  double last_output_{0.0};
  bool has_last_output_{false};
};

using PID = Pid;
using PIDConfig = PidConfig;
using PIDResult = PidResult;

}  // namespace savo_control
