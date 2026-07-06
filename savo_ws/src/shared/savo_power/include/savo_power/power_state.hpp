#ifndef SAVO_POWER__POWER_STATE_HPP_
#define SAVO_POWER__POWER_STATE_HPP_

#include <string_view>

namespace savo_power
{

enum class PowerState
{
  OK,
  LOW,
  CRITICAL,
  CHARGING,
  FULL,
  ERROR,
  STALE,
  UNKNOWN
};

inline constexpr std::string_view to_string(PowerState state)
{
  switch (state) {
    case PowerState::OK:
      return "ok";
    case PowerState::LOW:
      return "low";
    case PowerState::CRITICAL:
      return "critical";
    case PowerState::CHARGING:
      return "charging";
    case PowerState::FULL:
      return "full";
    case PowerState::ERROR:
      return "error";
    case PowerState::STALE:
      return "stale";
    case PowerState::UNKNOWN:
      return "unknown";
  }

  return "unknown";
}

inline constexpr bool is_fault_state(PowerState state)
{
  return state == PowerState::LOW ||
         state == PowerState::CRITICAL ||
         state == PowerState::ERROR ||
         state == PowerState::STALE ||
         state == PowerState::UNKNOWN;
}

inline constexpr bool is_safe_state(PowerState state)
{
  return state == PowerState::OK ||
         state == PowerState::CHARGING ||
         state == PowerState::FULL;
}

inline constexpr bool requires_operator_attention(PowerState state)
{
  return state == PowerState::LOW ||
         state == PowerState::CRITICAL ||
         state == PowerState::ERROR ||
         state == PowerState::STALE;
}

inline constexpr bool allows_normal_operation(PowerState state)
{
  return state == PowerState::OK ||
         state == PowerState::CHARGING ||
         state == PowerState::FULL;
}

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_STATE_HPP_
