#ifndef SAVO_POWER__POWER_HEALTH_HPP_
#define SAVO_POWER__POWER_HEALTH_HPP_

#include "savo_power/power_state.hpp"
#include "savo_power/power_types.hpp"
#include "savo_power/visibility_control.hpp"

#include <string>

namespace savo_power
{

struct PowerHealthConfig
{
  double stale_timeout_s{5.0};

  bool missing_status_is_error{true};
  bool stale_status_is_error{true};
  bool unknown_state_is_error{true};
};

struct PowerHealthInput
{
  bool seen{false};
  PowerState state{PowerState::UNKNOWN};

  double age_s{0.0};

  std::string status_text{};
};

struct PowerHealthResult
{
  PowerHealthLevel level{PowerHealthLevel::UNKNOWN};
  PowerState state{PowerState::UNKNOWN};

  bool missing{true};
  bool stale{false};

  std::string reason{};
  std::string text{};
};

class SAVO_POWER_PUBLIC PowerHealth
{
public:
  explicit PowerHealth(
    PowerHealthConfig config = PowerHealthConfig{});

  const PowerHealthConfig & config() const;

  void set_config(const PowerHealthConfig & config);

  PowerHealthResult evaluate(
    const PowerHealthInput & input) const;

  PowerHealthResult evaluate_status_text(
    bool seen,
    double age_s,
    const std::string & status_text) const;

  static PowerState state_from_status_text(
    const std::string & status_text);

private:
  PowerHealthLevel level_for_state(
    PowerState state) const;

  std::string make_health_text(
    const PowerHealthResult & result,
    const PowerHealthInput & input) const;

  PowerHealthConfig config_{};
};

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_HEALTH_HPP_
