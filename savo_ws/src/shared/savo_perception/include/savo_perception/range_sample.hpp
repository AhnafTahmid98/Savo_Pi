#ifndef SAVO_PERCEPTION__RANGE_SAMPLE_HPP_
#define SAVO_PERCEPTION__RANGE_SAMPLE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

using SteadyTimePoint = std::chrono::steady_clock::time_point;

struct SAVO_PERCEPTION_PUBLIC RangeSample
{
  std::string sensor_name;
  std::optional<double> distance_m;
  SteadyTimePoint stamp{std::chrono::steady_clock::now()};
  bool valid{false};
  std::string source;
  std::string error;

  [[nodiscard]] double age_s(
    const SteadyTimePoint & now = std::chrono::steady_clock::now()) const
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
    return valid &&
      distance_m.has_value() &&
      std::isfinite(*distance_m) &&
      !stale(stale_timeout_s, now);
  }
};

struct SAVO_PERCEPTION_PUBLIC RangeSnapshot
{
  RangeSample depth_front;
  RangeSample tof_left;
  RangeSample tof_right;
  RangeSample ultrasonic_front;

  [[nodiscard]] std::vector<RangeSample> all_samples() const
  {
    return {
      depth_front,
      tof_left,
      tof_right,
      ultrasonic_front,
    };
  }

  [[nodiscard]] std::map<std::string, double> front_candidates(
    const double stale_timeout_s,
    const bool use_depth_front = true,
    const SteadyTimePoint & now = std::chrono::steady_clock::now()) const
  {
    std::map<std::string, double> out;

    if (use_depth_front && depth_front.usable(stale_timeout_s, now)) {
      out[depth_front.sensor_name] = *depth_front.distance_m;
    }

    if (ultrasonic_front.usable(stale_timeout_s, now)) {
      out[ultrasonic_front.sensor_name] = *ultrasonic_front.distance_m;
    }

    return out;
  }

  [[nodiscard]] std::map<std::string, double> side_candidates(
    const double stale_timeout_s,
    const SteadyTimePoint & now = std::chrono::steady_clock::now()) const
  {
    std::map<std::string, double> out;

    if (tof_left.usable(stale_timeout_s, now)) {
      out[tof_left.sensor_name] = *tof_left.distance_m;
    }

    if (tof_right.usable(stale_timeout_s, now)) {
      out[tof_right.sensor_name] = *tof_right.distance_m;
    }

    return out;
  }

  [[nodiscard]] std::optional<double> min_front_m(
    const double stale_timeout_s,
    const bool use_depth_front = true,
    const SteadyTimePoint & now = std::chrono::steady_clock::now()) const
  {
    return min_from_map(front_candidates(stale_timeout_s, use_depth_front, now));
  }

  [[nodiscard]] std::optional<double> min_side_m(
    const double stale_timeout_s,
    const SteadyTimePoint & now = std::chrono::steady_clock::now()) const
  {
    return min_from_map(side_candidates(stale_timeout_s, now));
  }

  [[nodiscard]] std::vector<std::string> stale_sensors(
    const double stale_timeout_s,
    const SteadyTimePoint & now = std::chrono::steady_clock::now()) const
  {
    std::vector<std::string> out;

    for (const auto & sample : all_samples()) {
      if (sample.stale(stale_timeout_s, now)) {
        out.push_back(sample.sensor_name);
      }
    }

    return out;
  }

  [[nodiscard]] std::vector<std::string> invalid_sensors() const
  {
    std::vector<std::string> out;

    for (const auto & sample : all_samples()) {
      if (!sample.valid || !sample.distance_m.has_value()) {
        out.push_back(sample.sensor_name);
      }
    }

    return out;
  }

private:
  static std::optional<double> min_from_map(const std::map<std::string, double> & values)
  {
    if (values.empty()) {
      return std::nullopt;
    }

    auto best = values.begin()->second;

    for (const auto & item : values) {
      best = std::min(best, item.second);
    }

    return best;
  }
};

inline bool is_valid_distance(
  const double distance_m,
  const double min_m,
  const double max_m)
{
  return std::isfinite(distance_m) && distance_m >= min_m && distance_m <= max_m;
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

inline RangeSample make_valid_range_sample(
  std::string sensor_name,
  const double distance_m,
  std::string source = {})
{
  return make_range_sample(
    std::move(sensor_name),
    distance_m,
    true,
    std::move(source),
    "");
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

#endif  // SAVO_PERCEPTION__RANGE_SAMPLE_HPP_