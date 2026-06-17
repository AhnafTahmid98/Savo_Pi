#pragma once

#include "savo_base/base_types.hpp"

namespace savo_base
{

class MecanumMixer
{
public:
  explicit MecanumMixer(const BaseDriverConfig & config);

  void set_config(const BaseDriverConfig & config);
  WheelDuty mix(double vx, double vy, double wz) const;

private:
  static double clamp(double value, double min_value, double max_value);
  static int clamp_duty(int value, int max_duty);

  WheelDuty apply_breakaway(const WheelDuty & duty) const;

  BaseDriverConfig config_;
};

}  // namespace savo_base
