#ifndef SAVO_PERCEPTION__TYPES_HPP_
#define SAVO_PERCEPTION__TYPES_HPP_

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

using SteadyTimePoint = std::chrono::steady_clock::time_point;

enum class SensorStatus : std::uint8_t
{
  kOk = 0,
  kStale = 1,
  kError = 2,
};

enum class SafetyStatus : std::uint8_t
{
  kOk = 0,
  kSlow = 1,
  kSafetyStop = 2,
  kError = 3,
};

struct SAVO_PERCEPTION_PUBLIC RangeSample
{
  std::string sensor_name;
  std::optional<double> distance_m;
  SteadyTimePoint stamp{std::chrono::steady_clock::now()};
  bool valid{false};
  std::string source;
  std::string error;

  [[nodiscard]] double age_s(const SteadyTimePoint & now = std::chrono::steady_clock::now()) const
  {
    const auto dt = std::chrono::duration<double>(now - stamp);
    return std::max(0.0, dt.count());
  }

  [[nodiscard]] bool stale(
    const double stale_timeout_s,
    const SteadyTimePoint & now = std::chrono::steady_clock::now()) const
  {
    return age_s(now) > stale_timeout_s;
  }

  [[nodiscard]] bool usable(
    const double stale_timeout_s,
    const SteadyTimePoint & now = std::chrono::steady_clock::now()) const
  {
    return valid && distance_m.has_value() && std::isfinite(*distance_m) && !stale(stale_timeout_s, now);
  }
};

struct SAVO_PERCEPTION_PUBLIC RangeSnapshot
{
  RangeSample depth_front;
  RangeSample tof_left;
  RangeSample tof_right;
  RangeSample ultrasonic_front;
};

struct SAVO_PERCEPTION_PUBLIC SensorHealth
{
  std::string sensor_name;
  SensorStatus status{SensorStatus::kError};
  bool ok{false};
  bool stale{true};
  bool valid{false};
  std::optional<double> last_distance_m;
  double age_s{0.0};
  std::string error;
  std::string source;
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
};

struct SAVO_PERCEPTION_PUBLIC SafetyState
{
  SafetyDecision active_decision;
  std::uint64_t update_count{0};
  int stop_count{0};
  int clear_count{0};
};

inline bool valid_distance(
  const double distance_m,
  const double min_m,
  const double max_m)
{
  return std::isfinite(distance_m) && distance_m >= min_m && distance_m <= max_m;
}

inline double clamp_value(const double value, const double min_value, const double max_value)
{
  const auto bounds = std::minmax(min_value, max_value);
  return std::clamp(value, bounds.first, bounds.second);
}

inline double clamp_slowdown(const double value, const double min_value, const double max_value)
{
  return clamp_value(value, min_value, max_value);
}

inline const char * to_string(const SensorStatus status)
{
  switch (status) {
    case SensorStatus::kOk:
      return "OK";
    case SensorStatus::kStale:
      return "STALE";
    case SensorStatus::kError:
      return "ERROR";
  }

  return "ERROR";
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

inline RangeSample make_range_sample(
  std::string sensor_name,
  const std::optional<double> distance_m,
  const bool valid,
  std::string source = {},
  std::string error = {})
{
  return RangeSample{
    std::move(sensor_name),
    distance_m,
    std::chrono::steady_clock::now(),
    valid,
    std::move(source),
    std::move(error)};
}

inline RangeSample make_invalid_range_sample(
  std::string sensor_name,
  std::string error,
  std::string source = {})
{
  return make_range_sample(
    std::move(sensor_name),
    std::nullopt,
    false,
    std::move(source),
    std::move(error));
}

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__TYPES_HPP_