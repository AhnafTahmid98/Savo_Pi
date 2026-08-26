#pragma once

#include <array>

#include "savo_localization/encoder_state.hpp"

namespace savo_localization
{

constexpr double TWO_PI = 6.28318530717958647692;

constexpr std::array<const char *, 4> WHEEL_JOINT_NAMES = {
  "wheel_fl_link_joint",
  "wheel_fr_link_joint",
  "wheel_rl_link_joint",
  "wheel_rr_link_joint",
};

struct WheelJointState
{
  std::array<double, 4> position_rad{};
  std::array<double, 4> velocity_rad_s{};
};

WheelJointState wheel_joint_state_from_encoder_sample(
  const EncoderSample & sample,
  int counts_per_wheel_rev);

}  // namespace savo_localization
