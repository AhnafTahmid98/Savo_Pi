#include "savo_base/mecanum_mixer.hpp"

#include <algorithm>
#include <cmath>

namespace savo_base
{

MecanumMixer::MecanumMixer(const BaseDriverConfig & config)
: config_(config)
{
}

void MecanumMixer::set_config(const BaseDriverConfig & config)
{
  config_ = config;
}

WheelDuty MecanumMixer::mix(const double vx, const double vy, const double wz) const
{
  const double vx_limit = std::max(config_.velocity.vx, 1e-6);
  const double vy_limit = std::max(config_.velocity.vy, 1e-6);
  const double wz_limit = std::max(config_.velocity.wz, 1e-6);

  const double x =
    static_cast<double>(config_.signs.forward) *
    clamp(vx, -vx_limit, vx_limit) / vx_limit;

  const double y =
    static_cast<double>(config_.signs.strafe) *
    clamp(vy, -vy_limit, vy_limit) / vy_limit;

  const double w =
    static_cast<double>(config_.signs.rotate) *
    config_.turn_gain *
    clamp(wz, -wz_limit, wz_limit) / wz_limit;

  double fl = x - y - w;
  double rl = x + y - w;
  double fr = x + y + w;
  double rr = x - y + w;

  const double scale = std::max(
    1.0,
    std::max(
      std::max(std::abs(fl), std::abs(rl)),
      std::max(std::abs(fr), std::abs(rr))));

  fl /= scale;
  rl /= scale;
  fr /= scale;
  rr /= scale;

  const int max_duty = std::clamp(config_.motor.max_duty, 0, 4095);

  WheelDuty duty{
    clamp_duty(static_cast<int>(std::lround(fl * max_duty)), max_duty),
    clamp_duty(static_cast<int>(std::lround(rl * max_duty)), max_duty),
    clamp_duty(static_cast<int>(std::lround(fr * max_duty)), max_duty),
    clamp_duty(static_cast<int>(std::lround(rr * max_duty)), max_duty)
  };

  return apply_breakaway(duty);
}

double MecanumMixer::clamp(
  const double value,
  const double min_value,
  const double max_value)
{
  return std::min(std::max(value, min_value), max_value);
}

int MecanumMixer::clamp_duty(const int value, const int max_duty)
{
  return std::clamp(value, -max_duty, max_duty);
}

WheelDuty MecanumMixer::apply_breakaway(const WheelDuty & duty) const
{
  if (!config_.motor.enable_breakaway_compensation) {
    return duty;
  }

  const int max_duty = std::clamp(config_.motor.max_duty, 0, 4095);
  const int min_motion = std::clamp(config_.motor.min_motion_duty, 0, max_duty);
  const int trigger = std::clamp(config_.motor.breakaway_trigger_duty, 0, min_motion);

  if (min_motion <= 0) {
    return duty;
  }

  const auto apply_one = [min_motion, trigger, max_duty](const int value) -> int {
      const int abs_value = std::abs(value);

      if (abs_value == 0) {
        return 0;
      }

      if (abs_value < trigger) {
        return 0;
      }

      if (abs_value < min_motion) {
        return value > 0 ? min_motion : -min_motion;
      }

      return std::clamp(value, -max_duty, max_duty);
    };

  return WheelDuty{
    apply_one(duty.fl),
    apply_one(duty.rl),
    apply_one(duty.fr),
    apply_one(duty.rr)
  };
}

}  // namespace savo_base
