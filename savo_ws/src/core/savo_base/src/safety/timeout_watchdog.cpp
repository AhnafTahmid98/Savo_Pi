#include "savo_base/timeout_watchdog.hpp"

#include <algorithm>
#include <cmath>

namespace savo_base
{

TimeoutWatchdog::TimeoutWatchdog(const double timeout_s)
{
  set_timeout(timeout_s);
}

void TimeoutWatchdog::set_timeout(const double timeout_s)
{
  if (std::isfinite(timeout_s) && timeout_s > 0.0) {
    timeout_s_ = std::clamp(timeout_s, 0.05, 5.0);
    return;
  }

  timeout_s_ = 0.30;
}

double TimeoutWatchdog::timeout() const
{
  return timeout_s_;
}

WatchdogStatus TimeoutWatchdog::evaluate(
  const bool has_command,
  const double command_age_s) const
{
  WatchdogStatus status{};
  status.has_command = has_command;
  status.age_s = command_age_s;
  status.timeout_s = timeout_s_;

  if (!has_command) {
    status.stale = true;
    status.tripped = true;
    status.reason = "no_command";
    return status;
  }

  if (!std::isfinite(command_age_s)) {
    status.stale = true;
    status.tripped = true;
    status.reason = "invalid_command_age";
    return status;
  }

  if (command_age_s > timeout_s_) {
    status.stale = true;
    status.tripped = true;
    status.reason = "command_timeout";
    return status;
  }

  status.stale = false;
  status.tripped = false;
  status.reason = "command_fresh";
  return status;
}

}  // namespace savo_base
