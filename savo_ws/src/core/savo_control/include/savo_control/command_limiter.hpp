#pragma once

#include <cmath>

#include "savo_control/control_math.hpp"

namespace savo_control
{

struct TwistCommand
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};

  bool empty() const
  {
    return ControlMath::near_zero(vx) &&
           ControlMath::near_zero(vy) &&
           ControlMath::near_zero(wz);
  }

  bool moving(const double epsilon = 1.0e-4) const
  {
    const double eps = std::abs(epsilon);

    return std::abs(vx) > eps ||
           std::abs(vy) > eps ||
           std::abs(wz) > eps;
  }
};

struct CommandLimits
{
  double max_vx{0.25};
  double max_vy{0.25};
  double max_wz{0.60};

  double min_vx{-0.25};
  double min_vy{-0.25};
  double min_wz{-0.60};

  double deadband_vx{0.0};
  double deadband_vy{0.0};
  double deadband_wz{0.0};

  bool use_symmetric_limits{true};

  CommandLimits sanitized() const
  {
    CommandLimits out;

    out.max_vx = std::abs(ControlMath::finite_or_zero(max_vx));
    out.max_vy = std::abs(ControlMath::finite_or_zero(max_vy));
    out.max_wz = std::abs(ControlMath::finite_or_zero(max_wz));

    out.min_vx = ControlMath::finite_or_zero(min_vx);
    out.min_vy = ControlMath::finite_or_zero(min_vy);
    out.min_wz = ControlMath::finite_or_zero(min_wz);

    out.deadband_vx = std::abs(ControlMath::finite_or_zero(deadband_vx));
    out.deadband_vy = std::abs(ControlMath::finite_or_zero(deadband_vy));
    out.deadband_wz = std::abs(ControlMath::finite_or_zero(deadband_wz));

    out.use_symmetric_limits = use_symmetric_limits;

    if (out.use_symmetric_limits) {
      out.min_vx = -out.max_vx;
      out.min_vy = -out.max_vy;
      out.min_wz = -out.max_wz;
      return out;
    }

    if (out.min_vx > out.max_vx) {
      const double tmp = out.min_vx;
      out.min_vx = out.max_vx;
      out.max_vx = tmp;
    }

    if (out.min_vy > out.max_vy) {
      const double tmp = out.min_vy;
      out.min_vy = out.max_vy;
      out.max_vy = tmp;
    }

    if (out.min_wz > out.max_wz) {
      const double tmp = out.min_wz;
      out.min_wz = out.max_wz;
      out.max_wz = tmp;
    }

    return out;
  }
};

class CommandLimiter
{
public:
  CommandLimiter() = default;

  explicit CommandLimiter(const CommandLimits & limits)
  : limits_(limits.sanitized())
  {
  }

  void set_limits(const CommandLimits & limits)
  {
    limits_ = limits.sanitized();
  }

  const CommandLimits & limits() const
  {
    return limits_;
  }

  TwistCommand limit(const TwistCommand & input) const
  {
    const CommandLimits safe = limits_.sanitized();

    TwistCommand out;
    out.vx = ControlMath::finite_or_zero(input.vx);
    out.vy = ControlMath::finite_or_zero(input.vy);
    out.wz = ControlMath::finite_or_zero(input.wz);

    out.vx = ControlMath::clamp(out.vx, safe.min_vx, safe.max_vx);
    out.vy = ControlMath::clamp(out.vy, safe.min_vy, safe.max_vy);
    out.wz = ControlMath::clamp(out.wz, safe.min_wz, safe.max_wz);

    out.vx = ControlMath::apply_deadband(out.vx, safe.deadband_vx);
    out.vy = ControlMath::apply_deadband(out.vy, safe.deadband_vy);
    out.wz = ControlMath::apply_deadband(out.wz, safe.deadband_wz);

    return out;
  }

  TwistCommand operator()(const TwistCommand & input) const
  {
    return limit(input);
  }

  static TwistCommand zero()
  {
    return TwistCommand{};
  }

private:
  CommandLimits limits_{};
};

inline TwistCommand make_twist_command(
  const double vx,
  const double vy,
  const double wz)
{
  return TwistCommand{
    ControlMath::finite_or_zero(vx),
    ControlMath::finite_or_zero(vy),
    ControlMath::finite_or_zero(wz)};
}

inline TwistCommand limit_command(
  const TwistCommand & command,
  const CommandLimits & limits)
{
  return CommandLimiter(limits).limit(command);
}

}  // namespace savo_control
