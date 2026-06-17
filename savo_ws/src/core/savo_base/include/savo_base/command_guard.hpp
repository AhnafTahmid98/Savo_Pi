#pragma once

#include "savo_base/base_types.hpp"

namespace savo_base
{

struct GuardedCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  bool valid{true};
};

class CommandGuard
{
public:
  explicit CommandGuard(const VelocityLimits & limits);

  void set_limits(const VelocityLimits & limits);
  GuardedCommand apply(double vx, double vy, double wz) const;

private:
  static double safe_limit(double value);
  static double clamp(double value, double min_value, double max_value);

  VelocityLimits limits_{};
};

}  // namespace savo_base
