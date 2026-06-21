#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "savo_control/control_math.hpp"
#include "savo_control/control_mode.hpp"

namespace savo_control
{

struct ControlModeManagerConfig
{
  ControlMode startup_mode{ControlMode::STOP};
  ControlMode fallback_mode{ControlMode::STOP};

  double request_timeout_s{1.0};
  double recovery_latch_timeout_s{2.0};
  double manual_override_timeout_s{1.5};

  bool allow_manual{true};
  bool allow_auto{true};
  bool allow_nav{true};
  bool allow_recovery{true};

  bool safety_stop_forces_stop{true};
  bool external_stop_forces_stop{true};
  bool recovery_active_forces_recovery{true};

  ControlModeManagerConfig sanitized() const
  {
    ControlModeManagerConfig out = *this;

    if (!std::isfinite(out.request_timeout_s) || out.request_timeout_s < 0.0) {
      out.request_timeout_s = 1.0;
    }

    if (
      !std::isfinite(out.recovery_latch_timeout_s) ||
      out.recovery_latch_timeout_s < 0.0)
    {
      out.recovery_latch_timeout_s = 2.0;
    }

    if (
      !std::isfinite(out.manual_override_timeout_s) ||
      out.manual_override_timeout_s < 0.0)
    {
      out.manual_override_timeout_s = 1.5;
    }

    if (!is_allowed_mode(out.startup_mode)) {
      out.startup_mode = ControlMode::STOP;
    }

    if (!is_allowed_mode(out.fallback_mode)) {
      out.fallback_mode = ControlMode::STOP;
    }

    return out;
  }

  bool is_allowed_mode(const ControlMode mode) const
  {
    if (mode == ControlMode::STOP) {
      return true;
    }

    if (mode == ControlMode::MANUAL) {
      return allow_manual;
    }

    if (mode == ControlMode::AUTO) {
      return allow_auto;
    }

    if (mode == ControlMode::NAV) {
      return allow_nav;
    }

    if (mode == ControlMode::RECOVERY) {
      return allow_recovery;
    }

    return false;
  }
};

struct ControlModeRequest
{
  ControlMode mode{ControlMode::STOP};
  ControlModeReason reason{ControlModeReason::REQUESTED};

  std::string source{"unknown"};
  double stamp_s{0.0};

  bool force{false};
  bool valid{true};

  ControlModeRequest sanitized(const ControlModeManagerConfig & config) const
  {
    ControlModeRequest out = *this;

    out.stamp_s = ControlMath::finite_or_zero(out.stamp_s);

    if (out.source.empty()) {
      out.source = "unknown";
    }

    if (!out.force && !config.is_allowed_mode(out.mode)) {
      out.mode = config.fallback_mode;
      out.reason = ControlModeReason::NO_SOURCE_ALLOWED;
      out.valid = false;
    }

    return out;
  }
};

struct ControlModeManagerState
{
  ControlMode mode{ControlMode::STOP};
  ControlMode previous_mode{ControlMode::STOP};
  ControlModeReason reason{ControlModeReason::STARTUP};

  std::string source{"startup"};

  bool safety_stop_active{false};
  bool external_stop_active{false};
  bool recovery_active{false};
  bool manual_override_active{false};
  bool request_stale{false};

  double last_update_s{0.0};
  double last_request_s{0.0};
  double last_recovery_s{0.0};
  double last_manual_override_s{0.0};
};

class ControlModeManager
{
public:
  ControlModeManager()
  : config_(ControlModeManagerConfig{}.sanitized())
  {
    reset(0.0);
  }

  explicit ControlModeManager(const ControlModeManagerConfig & config)
  : config_(config.sanitized())
  {
    reset(0.0);
  }

  void set_config(const ControlModeManagerConfig & config)
  {
    config_ = config.sanitized();

    if (!config_.is_allowed_mode(state_.mode)) {
      set_mode(config_.fallback_mode, ControlModeReason::NO_SOURCE_ALLOWED, "config");
    }
  }

  const ControlModeManagerConfig & config() const
  {
    return config_;
  }

  const ControlModeManagerState & state() const
  {
    return state_;
  }

  void reset(const double now_s = 0.0)
  {
    const double now = ControlMath::finite_or_zero(now_s);

    state_ = ControlModeManagerState{};
    state_.mode = config_.startup_mode;
    state_.previous_mode = config_.startup_mode;
    state_.reason = ControlModeReason::STARTUP;
    state_.source = "startup";
    state_.last_update_s = now;
    state_.last_request_s = now;
    state_.last_recovery_s = now;
    state_.last_manual_override_s = now;

    last_request_ = ControlModeRequest{};
    last_request_.mode = config_.startup_mode;
    last_request_.reason = ControlModeReason::STARTUP;
    last_request_.source = "startup";
    last_request_.stamp_s = now;
    last_request_.valid = true;
  }

  bool request_mode(
    const ControlMode mode,
    const double now_s,
    const std::string & source = "request",
    const bool force = false)
  {
    ControlModeRequest request;
    request.mode = mode;
    request.reason = ControlModeReason::REQUESTED;
    request.source = source;
    request.stamp_s = now_s;
    request.force = force;
    request.valid = true;

    return request_mode(request);
  }

  bool request_mode(
    const std::string & mode_text, const double now_s,
    const std::string & source = "request")
  {
    ControlMode parsed = config_.fallback_mode;
    const bool parsed_ok = try_parse_control_mode(mode_text, parsed);

    ControlModeRequest request;
    request.mode = parsed;
    request.reason = parsed_ok ? ControlModeReason::REQUESTED : ControlModeReason::UNKNOWN;
    request.source = source;
    request.stamp_s = now_s;
    request.valid = parsed_ok;

    return request_mode(request);
  }

  bool request_mode(const ControlModeRequest & request)
  {
    const auto safe_request = request.sanitized(config_);
    last_request_ = safe_request;
    state_.last_request_s = safe_request.stamp_s;

    if (!safe_request.valid && !safe_request.force) {
      return false;
    }

    if (!safe_request.force && !config_.is_allowed_mode(safe_request.mode)) {
      return false;
    }

    set_mode(safe_request.mode, safe_request.reason, safe_request.source);
    return true;
  }

  void set_safety_stop(const bool active)
  {
    state_.safety_stop_active = active;
    if (active && config_.safety_stop_forces_stop) {
      set_mode(ControlMode::STOP, ControlModeReason::SAFETY_STOP_ACTIVE, "safety_stop");
    }
  }

  void set_external_stop(const bool active)
  {
    state_.external_stop_active = active;
    if (active && config_.external_stop_forces_stop) {
      set_mode(ControlMode::STOP, ControlModeReason::EXTERNAL_STOP, "external_stop");
    }
  }

  void set_recovery_active(const bool active, const double now_s)
  {
    state_.recovery_active = active;
    if (active) {
      state_.last_recovery_s = ControlMath::finite_or_zero(now_s);

      if (config_.recovery_active_forces_recovery && config_.allow_recovery) {
        set_mode(ControlMode::RECOVERY, ControlModeReason::RECOVERY_ACTIVE, "recovery");
      }
    }
  }

  void set_manual_override(const bool active, const double now_s)
  {
    state_.manual_override_active = active;
    if (active) {
      state_.last_manual_override_s = ControlMath::finite_or_zero(now_s);

      if (config_.allow_manual) {
        set_mode(ControlMode::MANUAL, ControlModeReason::MANUAL_OVERRIDE, "manual_override");
      }
    }
  }

  ControlMode update(const double now_s)
  {
    const double now = ControlMath::finite_or_zero(now_s);
    state_.last_update_s = now;

    state_.request_stale = is_request_stale(now);

    if (state_.safety_stop_active && config_.safety_stop_forces_stop) {
      set_mode(ControlMode::STOP, ControlModeReason::SAFETY_STOP_ACTIVE, "safety_stop");
      return state_.mode;
    }

    if (state_.external_stop_active && config_.external_stop_forces_stop) {
      set_mode(ControlMode::STOP, ControlModeReason::EXTERNAL_STOP, "external_stop");
      return state_.mode;
    }

    if (state_.recovery_active && config_.recovery_active_forces_recovery &&
      config_.allow_recovery)
    {
      set_mode(ControlMode::RECOVERY, ControlModeReason::RECOVERY_ACTIVE, "recovery");
      return state_.mode;
    }

    if (
      state_.mode == ControlMode::RECOVERY &&
      !state_.recovery_active &&
      recovery_latch_expired(now))
    {
      set_mode(ControlMode::STOP, ControlModeReason::TIMEOUT, "recovery_latch_timeout");
      return state_.mode;
    }

    if (
      state_.mode == ControlMode::MANUAL &&
      state_.manual_override_active &&
      manual_override_expired(now))
    {
      state_.manual_override_active = false;
      set_mode(ControlMode::STOP, ControlModeReason::TIMEOUT, "manual_override_timeout");
      return state_.mode;
    }

    if (state_.request_stale && is_motion_mode(state_.mode)) {
      set_mode(config_.fallback_mode, ControlModeReason::TIMEOUT, "request_timeout");
      return state_.mode;
    }

    if (!config_.is_allowed_mode(state_.mode)) {
      set_mode(config_.fallback_mode, ControlModeReason::NO_SOURCE_ALLOWED, "mode_not_allowed");
      return state_.mode;
    }

    return state_.mode;
  }

  ControlMode mode() const
  {
    return state_.mode;
  }

  ControlMode previous_mode() const
  {
    return state_.previous_mode;
  }

  ControlModeReason reason() const
  {
    return state_.reason;
  }

  std::string source() const
  {
    return state_.source;
  }

  bool safety_stop_active() const
  {
    return state_.safety_stop_active;
  }

  bool recovery_active() const
  {
    return state_.recovery_active;
  }

  bool external_stop_active() const
  {
    return state_.external_stop_active;
  }

  bool manual_override_active() const
  {
    return state_.manual_override_active;
  }

  bool request_stale() const
  {
    return state_.request_stale;
  }

  bool allows_motion() const
  {
    if (state_.safety_stop_active || state_.external_stop_active) {
      return false;
    }

    return is_motion_mode(state_.mode);
  }

  std::string mux_mode_string() const
  {
    return plain_mux_mode_string(state_.mode);
  }

  std::string status_string() const
  {
    std::ostringstream ss;
    ss << "mode=" << to_string(state_.mode)
       << "; previous=" << to_string(state_.previous_mode)
       << "; reason=" << to_string(state_.reason)
       << "; source=" << state_.source
       << "; safety_stop=" << bool_text(state_.safety_stop_active)
       << "; external_stop=" << bool_text(state_.external_stop_active)
       << "; recovery_active=" << bool_text(state_.recovery_active)
       << "; manual_override=" << bool_text(state_.manual_override_active)
       << "; request_stale=" << bool_text(state_.request_stale)
       << "; mux_mode=" << mux_mode_string();

    return ss.str();
  }

private:
  void set_mode(
    const ControlMode mode,
    const ControlModeReason reason,
    const std::string & source)
  {
    if (state_.mode != mode) {
      state_.previous_mode = state_.mode;
    }

    state_.mode = mode;
    state_.reason = reason;
    state_.source = source.empty() ? "unknown" : source;
  }

  bool is_request_stale(const double now_s) const
  {
    if (config_.request_timeout_s <= 0.0) {
      return false;
    }

    const double age_s = now_s - state_.last_request_s;
    return age_s > config_.request_timeout_s;
  }

  bool recovery_latch_expired(const double now_s) const
  {
    if (config_.recovery_latch_timeout_s <= 0.0) {
      return false;
    }

    return (now_s - state_.last_recovery_s) > config_.recovery_latch_timeout_s;
  }

  bool manual_override_expired(const double now_s) const
  {
    if (config_.manual_override_timeout_s <= 0.0) {
      return false;
    }

    return (now_s - state_.last_manual_override_s) > config_.manual_override_timeout_s;
  }

  static const char * bool_text(const bool value)
  {
    return value ? "true" : "false";
  }

  ControlModeManagerConfig config_{};
  ControlModeManagerState state_{};
  ControlModeRequest last_request_{};
};

}  // namespace savo_control
