#include "savo_localization/encoder_state.hpp"

#include <set>
#include <stdexcept>

namespace savo_localization
{

bool WheelEncoderConfig::valid() const
{
  return wheel_name_valid(name) &&
         gpio_pin_valid(a_gpio) &&
         gpio_pin_valid(b_gpio) &&
         a_gpio != b_gpio;
}

std::array<WheelEncoderConfig, 4> EncoderHardwareConfig::wheels() const
{
  return {fl, fr, rl, rr};
}

bool EncoderHardwareConfig::valid() const
{
  if (gpiochip < -1) {
    return false;
  }

  if (poll_s <= 0.0 || debounce_s < 0.0) {
    return false;
  }

  const auto wheel_configs = wheels();
  std::set<int> pins;

  for (const auto & wheel : wheel_configs) {
    if (!wheel.valid()) {
      return false;
    }

    if (!pins.insert(wheel.a_gpio).second) {
      return false;
    }

    if (!pins.insert(wheel.b_gpio).second) {
      return false;
    }
  }

  return true;
}

void WheelEncoderState::update(
  std::int64_t new_count,
  double dt_s,
  double new_speed_mps,
  double stamp_s,
  std::uint64_t new_illegal_transitions)
{
  const double safe_dt_s = dt_s > 1e-9 ? dt_s : 1e-9;

  previous_count = count;
  count = new_count;
  delta_count = count - previous_count;

  counts_per_second = static_cast<double>(delta_count) / safe_dt_s;
  speed_mps = new_speed_mps;

  direction = direction_from_delta(delta_count);
  active = delta_count != 0;

  illegal_transitions = new_illegal_transitions;
  last_update_s = stamp_s;
}

bool WheelEncoderState::moving_forward() const
{
  return direction > 0;
}

bool WheelEncoderState::moving_reverse() const
{
  return direction < 0;
}

bool WheelEncoderState::stopped() const
{
  return direction == 0;
}

std::array<WheelEncoderState, 4> EncoderSample::wheels() const
{
  return {fl, fr, rl, rr};
}

int EncoderSample::active_wheel_count() const
{
  int active_count = 0;

  for (const auto & wheel : wheels()) {
    if (wheel.active) {
      ++active_count;
    }
  }

  return active_count;
}

std::uint64_t EncoderSample::total_illegal_transitions() const
{
  std::uint64_t total = 0;

  for (const auto & wheel : wheels()) {
    total += wheel.illegal_transitions;
  }

  return total;
}

int direction_from_delta(std::int64_t delta_count)
{
  if (delta_count > 0) {
    return 1;
  }

  if (delta_count < 0) {
    return -1;
  }

  return 0;
}

std::string wheel_id_to_name(WheelId wheel_id)
{
  switch (wheel_id) {
    case WheelId::FL:
      return WHEEL_FL;
    case WheelId::FR:
      return WHEEL_FR;
    case WheelId::RL:
      return WHEEL_RL;
    case WheelId::RR:
      return WHEEL_RR;
  }

  throw std::invalid_argument("Unsupported wheel id");
}

WheelId wheel_id_from_name(const std::string & name)
{
  if (name == WHEEL_FL) {
    return WheelId::FL;
  }

  if (name == WHEEL_FR) {
    return WheelId::FR;
  }

  if (name == WHEEL_RL) {
    return WheelId::RL;
  }

  if (name == WHEEL_RR) {
    return WheelId::RR;
  }

  throw std::invalid_argument("Unsupported wheel name: " + name);
}

bool wheel_name_valid(const std::string & name)
{
  return name == WHEEL_FL ||
         name == WHEEL_FR ||
         name == WHEEL_RL ||
         name == WHEEL_RR;
}

bool gpio_pin_valid(int gpio_pin)
{
  return gpio_pin >= 0;
}

std::map<std::string, std::int64_t> counts_to_map(const EncoderSample & sample)
{
  return {
    {WHEEL_FL, sample.fl.count},
    {WHEEL_FR, sample.fr.count},
    {WHEEL_RL, sample.rl.count},
    {WHEEL_RR, sample.rr.count},
  };
}

std::map<std::string, double> speeds_to_map(const EncoderSample & sample)
{
  return {
    {WHEEL_FL, sample.fl.speed_mps},
    {WHEEL_FR, sample.fr.speed_mps},
    {WHEEL_RL, sample.rl.speed_mps},
    {WHEEL_RR, sample.rr.speed_mps},
  };
}

}  // namespace savo_localization