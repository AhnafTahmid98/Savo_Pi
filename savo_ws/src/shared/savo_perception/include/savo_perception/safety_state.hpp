#ifndef SAVO_PERCEPTION__SAFETY_STATE_HPP_
#define SAVO_PERCEPTION__SAFETY_STATE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

enum class SafetyStatus : std::uint8_t
{
  kOk = 0,
  kSlow = 1,
  kSafetyStop = 2,
  kError = 3,
};

struct SAVO_PERCEPTION_PUBLIC SafetyDecision
{
  bool stop_required{true};
  double slowdown_factor{0.0};
  SafetyStatus status{SafetyStatus::kSafetyStop};
  std::string reason{"initial"};

  std::optional<double> front_distance_m;
  std::optional<double> side_distance_m;

  std::vector<std::string> stale_sensors;
  std::vector<std::string> invalid_sensors;

  std::string source{"safety_fusion"};
  std::chrono::steady_clock::time_point stamp{std::chrono::steady_clock::now()};

  [[nodiscard]] double age_s(
    const std::chrono::steady_clock::time_point & now = std::chrono::steady_clock::now()) const
  {
    const auto dt = std::chrono::duration<double>(now - stamp);
    return std::max(0.0, dt.count());
  }
};

struct SAVO_PERCEPTION_PUBLIC SafetyState
{
  SafetyDecision active_decision;
  SafetyDecision last_clear_decision;

  bool has_last_clear_decision{false};

  std::uint64_t update_count{0};
  int stop_count{0};
  int clear_count{0};

  [[nodiscard]] bool stop_required() const
  {
    return active_decision.stop_required;
  }

  [[nodiscard]] double slowdown_factor() const
  {
    return active_decision.slowdown_factor;
  }

  [[nodiscard]] SafetyStatus status() const
  {
    return active_decision.status;
  }

  [[nodiscard]] const std::string & reason() const
  {
    return active_decision.reason;
  }
};

inline double clamp_value(const double value, const double min_value, const double max_value)
{
  const auto bounds = std::minmax(min_value, max_value);
  return std::clamp(value, bounds.first, bounds.second);
}

inline double clamp_slowdown(
  const double value,
  const double min_value = 0.0,
  const double max_value = 1.0)
{
  return clamp_value(value, min_value, max_value);
}

inline const char * to_string(const SafetyStatus status)
{
  switch (status) {
    case SafetyStatus::kOk:
      return "OK";
    case SafetyStatus::kSlow:
      return "SLOW";
    case SafetyStatus::kSafetyStop:
      return "SAFETY_STOP";
    case SafetyStatus::kError:
      return "ERROR";
  }

  return "ERROR";
}

inline SafetyDecision make_clear_decision(std::string reason = "clear")
{
  SafetyDecision decision;
  decision.stop_required = false;
  decision.slowdown_factor = 1.0;
  decision.status = SafetyStatus::kOk;
  decision.reason = std::move(reason);
  decision.source = "safety_fusion";
  decision.stamp = std::chrono::steady_clock::now();
  return decision;
}

inline SafetyDecision make_slow_decision(
  const double slowdown_factor,
  std::string reason,
  const std::optional<double> & front_distance_m = std::nullopt,
  const std::optional<double> & side_distance_m = std::nullopt,
  const double slowdown_min = 0.0,
  const double slowdown_max = 1.0)
{
  SafetyDecision decision;
  decision.stop_required = false;
  decision.slowdown_factor = clamp_slowdown(slowdown_factor, slowdown_min, slowdown_max);
  decision.status = SafetyStatus::kSlow;
  decision.reason = std::move(reason);
  decision.front_distance_m = front_distance_m;
  decision.side_distance_m = side_distance_m;
  decision.source = "safety_fusion";
  decision.stamp = std::chrono::steady_clock::now();
  return decision;
}

inline SafetyDecision make_stop_decision(
  std::string reason,
  const std::optional<double> & front_distance_m = std::nullopt,
  const std::optional<double> & side_distance_m = std::nullopt,
  std::vector<std::string> stale_sensors = {},
  std::vector<std::string> invalid_sensors = {})
{
  SafetyDecision decision;
  decision.stop_required = true;
  decision.slowdown_factor = 0.0;
  decision.status = SafetyStatus::kSafetyStop;
  decision.reason = std::move(reason);
  decision.front_distance_m = front_distance_m;
  decision.side_distance_m = side_distance_m;
  decision.stale_sensors = std::move(stale_sensors);
  decision.invalid_sensors = std::move(invalid_sensors);
  decision.source = "safety_fusion";
  decision.stamp = std::chrono::steady_clock::now();
  return decision;
}

inline SafetyDecision make_error_decision(
  std::string reason,
  std::vector<std::string> stale_sensors = {},
  std::vector<std::string> invalid_sensors = {})
{
  SafetyDecision decision;
  decision.stop_required = true;
  decision.slowdown_factor = 0.0;
  decision.status = SafetyStatus::kError;
  decision.reason = std::move(reason);
  decision.stale_sensors = std::move(stale_sensors);
  decision.invalid_sensors = std::move(invalid_sensors);
  decision.source = "safety_fusion";
  decision.stamp = std::chrono::steady_clock::now();
  return decision;
}

inline SafetyState make_initial_safety_state()
{
  SafetyState state;
  state.active_decision = make_clear_decision("initial");
  state.has_last_clear_decision = true;
  state.last_clear_decision = state.active_decision;
  state.update_count = 0;
  state.stop_count = 0;
  state.clear_count = 0;
  return state;
}

inline SafetyState with_decision(
  SafetyState state,
  SafetyDecision decision,
  const int stop_count,
  const int clear_count)
{
  state.active_decision = std::move(decision);
  state.stop_count = stop_count;
  state.clear_count = clear_count;
  state.update_count += 1;

  if (!state.active_decision.stop_required) {
    state.last_clear_decision = state.active_decision;
    state.has_last_clear_decision = true;
  }

  return state;
}

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__SAFETY_STATE_HPP_