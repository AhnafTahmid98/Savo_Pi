#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "savo_control/control_math.hpp"

namespace savo_control
{

enum class WatchdogState
{
  OK,
  STALE,
  TIMEOUT,
  DISABLED
};

inline std::string to_string(const WatchdogState state)
{
  switch (state) {
    case WatchdogState::OK:
      return "OK";
    case WatchdogState::STALE:
      return "STALE";
    case WatchdogState::TIMEOUT:
      return "TIMEOUT";
    case WatchdogState::DISABLED:
      return "DISABLED";
  }

  return "TIMEOUT";
}

inline std::string watchdog_state_to_string(const WatchdogState state)
{
  return to_string(state);
}

inline bool watchdog_is_ok(const WatchdogState state)
{
  return state == WatchdogState::OK;
}

inline bool watchdog_is_faulted(const WatchdogState state)
{
  return state == WatchdogState::STALE ||
         state == WatchdogState::TIMEOUT ||
         state == WatchdogState::DISABLED;
}

struct WatchdogConfig
{
  double stale_timeout_s{0.50};
  double hard_timeout_s{1.00};

  bool enabled{true};
  bool fail_safe_on_never_seen{true};
  bool timeout_is_fault{true};

  WatchdogConfig sanitized() const
  {
    WatchdogConfig out = *this;

    if (!std::isfinite(out.stale_timeout_s) || out.stale_timeout_s < 0.0) {
      out.stale_timeout_s = 0.50;
    }

    if (!std::isfinite(out.hard_timeout_s) || out.hard_timeout_s < out.stale_timeout_s) {
      out.hard_timeout_s = std::max(1.00, out.stale_timeout_s);
    }

    return out;
  }
};

struct WatchdogStatus
{
  WatchdogState state{WatchdogState::TIMEOUT};

  bool ok{false};
  bool stale{true};
  bool timeout{true};
  bool enabled{true};
  bool seen{false};

  double age_s{0.0};
  double last_seen_s{0.0};
  double now_s{0.0};

  std::string name{"watchdog"};
  std::string reason{"never_seen"};

  std::string to_status_text() const
  {
    std::ostringstream ss;
    ss << "name=" << name
       << "; state=" << to_string(state)
       << "; ok=" << bool_text(ok)
       << "; stale=" << bool_text(stale)
       << "; timeout=" << bool_text(timeout)
       << "; enabled=" << bool_text(enabled)
       << "; seen=" << bool_text(seen)
       << "; age_s=" << age_s
       << "; reason=" << reason;

    return ss.str();
  }

private:
  static const char * bool_text(const bool value)
  {
    return value ? "true" : "false";
  }
};

class Watchdog
{
public:
  Watchdog()
  : config_(WatchdogConfig{}.sanitized())
  {
  }

  explicit Watchdog(const WatchdogConfig & config)
  : config_(config.sanitized())
  {
  }

  Watchdog(const std::string & name, const WatchdogConfig & config = WatchdogConfig{})
  : name_(name.empty() ? "watchdog" : name),
    config_(config.sanitized())
  {
  }

  void set_config(const WatchdogConfig & config)
  {
    config_ = config.sanitized();
  }

  const WatchdogConfig & config() const
  {
    return config_;
  }

  void set_name(const std::string & name)
  {
    name_ = name.empty() ? "watchdog" : name;
  }

  const std::string & name() const
  {
    return name_;
  }

  void reset()
  {
    last_seen_s_ = 0.0;
    has_seen_ = false;
  }

  void tick(const double now_s)
  {
    last_seen_s_ = ControlMath::finite_or_zero(now_s);
    has_seen_ = true;
  }

  void observe(const double now_s)
  {
    tick(now_s);
  }

  bool seen() const
  {
    return has_seen_;
  }

  double last_seen_s() const
  {
    return last_seen_s_;
  }

  double age_s(const double now_s) const
  {
    if (!has_seen_) {
      return 0.0;
    }

    return std::max(0.0, ControlMath::finite_or_zero(now_s) - last_seen_s_);
  }

  bool fresh(const double now_s) const
  {
    return status(now_s).ok;
  }

  bool stale(const double now_s) const
  {
    return status(now_s).stale;
  }

  bool timed_out(const double now_s) const
  {
    return status(now_s).timeout;
  }

  WatchdogStatus status(const double now_s) const
  {
    const auto cfg = config_.sanitized();
    const double safe_now = ControlMath::finite_or_zero(now_s);

    WatchdogStatus out;
    out.name = name_;
    out.enabled = cfg.enabled;
    out.seen = has_seen_;
    out.last_seen_s = last_seen_s_;
    out.now_s = safe_now;
    out.age_s = age_s(safe_now);

    if (!cfg.enabled) {
      out.state = WatchdogState::DISABLED;
      out.ok = false;
      out.stale = true;
      out.timeout = true;
      out.reason = "disabled";
      return out;
    }

    if (!has_seen_) {
      if (cfg.fail_safe_on_never_seen) {
        out.state = WatchdogState::TIMEOUT;
        out.ok = false;
        out.stale = true;
        out.timeout = true;
        out.reason = "never_seen";
      } else {
        out.state = WatchdogState::OK;
        out.ok = true;
        out.stale = false;
        out.timeout = false;
        out.reason = "never_seen_allowed";
      }

      return out;
    }

    if (out.age_s > cfg.hard_timeout_s) {
      out.state = WatchdogState::TIMEOUT;
      out.ok = false;
      out.stale = true;
      out.timeout = cfg.timeout_is_fault;
      out.reason = "hard_timeout";
      return out;
    }

    if (out.age_s > cfg.stale_timeout_s) {
      out.state = WatchdogState::STALE;
      out.ok = false;
      out.stale = true;
      out.timeout = false;
      out.reason = "stale";
      return out;
    }

    out.state = WatchdogState::OK;
    out.ok = true;
    out.stale = false;
    out.timeout = false;
    out.reason = "fresh";
    return out;
  }

  std::string status_string(const double now_s) const
  {
    return status(now_s).to_status_text();
  }

private:
  std::string name_{"watchdog"};
  WatchdogConfig config_{};
  double last_seen_s_{0.0};
  bool has_seen_{false};
};

}  // namespace savo_control
