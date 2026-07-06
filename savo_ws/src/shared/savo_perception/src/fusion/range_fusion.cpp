#include "savo_perception/range_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace savo_perception
{
namespace
{

std::optional<double> min_from_map(const std::map<std::string, double> & values)
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

std::vector<std::string> effective_required_sensors(const RangeFusionConfig & config)
{
  auto required = config.required_sensors;

  if (config.depth_front_required && !contains_sensor_name(required, "depth_front")) {
    required.push_back("depth_front");
  }

  return required;
}

std::vector<std::string> required_invalid_sensors(
  const std::vector<std::string> & invalid_sensors,
  const std::vector<std::string> & required_sensors)
{
  std::vector<std::string> out;

  for (const auto & sensor : invalid_sensors) {
    if (contains_sensor_name(required_sensors, sensor)) {
      out.push_back(sensor);
    }
  }

  return out;
}

bool in_stop_zone(const std::optional<double> & distance_m, const double stop_m)
{
  return distance_m.has_value() && std::isfinite(*distance_m) && *distance_m <= stop_m;
}

bool in_slow_zone(
  const std::optional<double> & distance_m,
  const double stop_m,
  const double slow_m)
{
  return distance_m.has_value() &&
    std::isfinite(*distance_m) &&
    *distance_m > stop_m &&
    *distance_m < slow_m;
}

}  // namespace

std::optional<double> min_optional(
  const std::optional<double> & a,
  const std::optional<double> & b)
{
  if (a.has_value() && b.has_value()) {
    return std::min(*a, *b);
  }

  if (a.has_value()) {
    return a;
  }

  return b;
}

double slowdown_from_distance(
  const std::optional<double> & distance_m,
  const double stop_m,
  const double slow_m,
  const double slowdown_min,
  const double slowdown_max)
{
  const auto min_value = std::min(slowdown_min, slowdown_max);
  const auto max_value = std::max(slowdown_min, slowdown_max);

  if (!distance_m.has_value() || !std::isfinite(*distance_m)) {
    return max_value;
  }

  if (*distance_m <= stop_m) {
    return min_value;
  }

  if (*distance_m >= slow_m || slow_m <= stop_m) {
    return max_value;
  }

  const auto ratio = (*distance_m - stop_m) / (slow_m - stop_m);
  return clamp_slowdown(min_value + ratio * (max_value - min_value), min_value, max_value);
}

bool contains_sensor_name(
  const std::vector<std::string> & names,
  const std::string & sensor_name)
{
  return std::find(names.begin(), names.end(), sensor_name) != names.end();
}

std::vector<std::string> required_stale_sensors(
  const std::vector<std::string> & stale_sensors,
  const std::vector<std::string> & required_sensors)
{
  std::vector<std::string> out;

  for (const auto & sensor : stale_sensors) {
    if (contains_sensor_name(required_sensors, sensor)) {
      out.push_back(sensor);
    }
  }

  return out;
}

std::vector<std::string> collect_stale_sensors(
  const RangeSnapshot & snapshot,
  const double stale_timeout_s)
{
  return snapshot.stale_sensors(stale_timeout_s);
}

std::vector<std::string> collect_invalid_sensors(const RangeSnapshot & snapshot)
{
  return snapshot.invalid_sensors();
}

std::map<std::string, double> front_candidates(
  const RangeSnapshot & snapshot,
  const RangeFusionConfig & config)
{
  std::map<std::string, double> out;

  if (config.use_depth_front && snapshot.depth_front.usable(config.stale_timeout_s)) {
    out[snapshot.depth_front.sensor_name] = *snapshot.depth_front.distance_m;
  }

  return out;
}

std::optional<double> ultrasonic_front_distance(
  const RangeSnapshot & snapshot,
  const RangeFusionConfig & config)
{
  if (snapshot.ultrasonic_front.usable(config.stale_timeout_s)) {
    return snapshot.ultrasonic_front.distance_m;
  }

  return std::nullopt;
}

std::map<std::string, double> side_candidates(
  const RangeSnapshot & snapshot,
  const RangeFusionConfig & config)
{
  return snapshot.side_candidates(config.stale_timeout_s);
}

RangeFusionResult fuse_range_snapshot(
  const RangeSnapshot & snapshot,
  const RangeFusionConfig & config)
{
  RangeFusionResult result;

  result.front_sources = front_candidates(snapshot, config);
  result.side_sources = side_candidates(snapshot, config);

  result.front_distance_m = min_from_map(result.front_sources);
  result.side_distance_m = min_from_map(result.side_sources);
  result.ultrasonic_front_distance_m = ultrasonic_front_distance(snapshot, config);

  result.stale_sensors = collect_stale_sensors(snapshot, config.stale_timeout_s);
  result.invalid_sensors = collect_invalid_sensors(snapshot);

  const auto required = effective_required_sensors(config);
  const auto required_stale = required_stale_sensors(result.stale_sensors, required);
  const auto required_invalid = required_invalid_sensors(result.invalid_sensors, required);

  if (config.fail_safe_on_stale && !required_stale.empty()) {
    result.decision = make_stop_decision(
      "required_sensor_stale",
      result.front_distance_m,
      result.side_distance_m,
      required_stale,
      result.invalid_sensors);
  } else if (!required_invalid.empty()) {
    result.decision = make_stop_decision(
      "required_sensor_invalid",
      result.front_distance_m,
      result.side_distance_m,
      result.stale_sensors,
      required_invalid);
  } else if (in_stop_zone(result.front_distance_m, config.front_stop_m)) {
    result.decision = make_stop_decision(
      "front_stop_zone",
      result.front_distance_m,
      result.side_distance_m,
      result.stale_sensors,
      result.invalid_sensors);
  } else if (in_stop_zone(result.ultrasonic_front_distance_m, config.ultrasonic_stop_m)) {
    result.decision = make_stop_decision(
      "ultrasonic_close_stop",
      result.front_distance_m,
      result.side_distance_m,
      result.stale_sensors,
      result.invalid_sensors);
  } else if (in_stop_zone(result.side_distance_m, config.side_stop_m)) {
    result.decision = make_stop_decision(
      "side_stop_zone",
      result.front_distance_m,
      result.side_distance_m,
      result.stale_sensors,
      result.invalid_sensors);
  } else if (
    in_slow_zone(result.front_distance_m, config.front_stop_m, config.front_slow_m) ||
    in_slow_zone(result.side_distance_m, config.side_stop_m, config.side_slow_m))
  {
    const auto front_slowdown = slowdown_from_distance(
      result.front_distance_m,
      config.front_stop_m,
      config.front_slow_m,
      config.slowdown_min,
      config.slowdown_max);

    const auto side_slowdown = slowdown_from_distance(
      result.side_distance_m,
      config.side_stop_m,
      config.side_slow_m,
      config.slowdown_min,
      config.slowdown_max);

    const auto slowdown = std::min(front_slowdown, side_slowdown);
    const auto reason = front_slowdown <= side_slowdown ? "front_slow_zone" : "side_slow_zone";

    result.decision = make_slow_decision(
      slowdown,
      reason,
      result.front_distance_m,
      result.side_distance_m,
      config.slowdown_min,
      config.slowdown_max);
  } else {
    result.decision = make_clear_decision("clear");
    result.decision.front_distance_m = result.front_distance_m;
    result.decision.side_distance_m = result.side_distance_m;
  }

  result.decision.stale_sensors = result.stale_sensors;
  result.decision.invalid_sensors = result.invalid_sensors;
  result.decision.source = "range_fusion";

  return result;
}

RangeFusion::RangeFusion(RangeFusionConfig config)
: config_(std::move(config))
{
}

const RangeFusionConfig & RangeFusion::config() const
{
  return config_;
}

void RangeFusion::set_config(RangeFusionConfig config)
{
  config_ = std::move(config);
}

RangeFusionResult RangeFusion::fuse(const RangeSnapshot & snapshot) const
{
  return fuse_range_snapshot(snapshot, config_);
}

}  // namespace savo_perception
