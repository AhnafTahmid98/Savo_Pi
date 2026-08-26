#include "savo_localization/wheel_joint_state.hpp"

#include <stdexcept>

namespace savo_localization
{

WheelJointState wheel_joint_state_from_encoder_sample(
  const EncoderSample & sample,
  int counts_per_wheel_rev)
{
  if (counts_per_wheel_rev <= 0) {
    throw std::invalid_argument("counts_per_wheel_rev must be > 0");
  }

  const double radians_per_count =
    TWO_PI / static_cast<double>(counts_per_wheel_rev);
  const std::array<WheelEncoderState, 4> wheels = {
    sample.fl,
    sample.fr,
    sample.rl,
    sample.rr,
  };

  WheelJointState state{};
  for (std::size_t index = 0; index < wheels.size(); ++index) {
    state.position_rad[index] =
      static_cast<double>(wheels[index].count) * radians_per_count;
    state.velocity_rad_s[index] =
      wheels[index].counts_per_second * radians_per_count;
  }

  return state;
}

}  // namespace savo_localization
