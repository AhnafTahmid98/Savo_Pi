#include "savo_base/base_safety.hpp"

#include <algorithm>
#include <cmath>

namespace savo_base
{

BaseSafety::BaseSafety(const double watchdog_timeout_s)
{
  set_watchdog_timeout(watchdog_timeout_s);
}

void BaseSafety::set_watchdog_timeout(const double watchdog_timeout_s)
{
  if (std::isfinite(watchdog_timeout_s) && watchdog_timeout_s > 0.0) {
    watchdog_timeout_s_ = watchdog_timeout_s;
  } else {
    watchdog_timeout_s_ = 0.30;
  }
}

double BaseSafety::watchdog_timeout() const
{
  return watchdog_timeout_s_;
}

SafetyDecision BaseSafety::evaluate(const SafetySnapshot & snapshot) const
{
  const double slowdown = clamp_slowdown(snapshot.slowdown_factor);

  if (snapshot.safety_stop) {
    return SafetyDecision{
      true,
      true,
      "safety_stop_active",
      slowdown
    };
  }

  if (!snapshot.has_command) {
    return SafetyDecision{
      true,
      false,
      "no_command_idle",
      slowdown
    };
  }

  const bool timeout =
    snapshot.command_stale ||
    !std::isfinite(snapshot.command_age_s) ||
    snapshot.command_age_s > watchdog_timeout_s_;

  if (timeout) {
    return SafetyDecision{
      true,
      false,
      "command_timeout",
      slowdown
    };
  }

  return SafetyDecision{
    false,
    false,
    "motion_allowed",
    slowdown
  };
}

double BaseSafety::clamp_slowdown(const double value)
{
  if (!std::isfinite(value)) {
    return 1.0;
  }

  return std::clamp(value, 0.0, 1.0);
}

}  // namespace savo_base
