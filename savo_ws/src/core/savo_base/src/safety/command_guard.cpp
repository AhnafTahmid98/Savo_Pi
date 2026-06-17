#include "savo_base/command_guard.hpp"

#include <algorithm>
#include <cmath>

namespace savo_base
{

CommandGuard::CommandGuard(const VelocityLimits & limits)
: limits_(limits)
{
}

void CommandGuard::set_limits(const VelocityLimits & limits)
{
  limits_ = limits;
}

GuardedCommand CommandGuard::apply(const double vx, const double vy, const double wz) const
{
  GuardedCommand command{};

  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
    command.valid = false;
    return command;
  }

  command.vx = clamp(vx, -safe_limit(limits_.vx), safe_limit(limits_.vx));
  command.vy = clamp(vy, -safe_limit(limits_.vy), safe_limit(limits_.vy));
  command.wz = clamp(wz, -safe_limit(limits_.wz), safe_limit(limits_.wz));
  command.valid = true;

  return command;
}

double CommandGuard::safe_limit(const double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return 0.01;
  }

  return value;
}

double CommandGuard::clamp(
  const double value,
  const double min_value,
  const double max_value)
{
  return std::min(std::max(value, min_value), max_value);
}

}  // namespace savo_base
