#ifndef SAVO_PERCEPTION__RANGE_FUSION_HPP_
#define SAVO_PERCEPTION__RANGE_FUSION_HPP_

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "savo_perception/types.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

struct SAVO_PERCEPTION_PUBLIC RangeFusionConfig
{
  double front_stop_m{0.35};
  double front_slow_m{0.80};

  double side_stop_m{0.12};
  double side_slow_m{0.25};

  double stale_timeout_s{0.30};
  bool fail_safe_on_stale{true};

  bool use_depth_front{true};
  bool depth_front_required{false};

  double slowdown_min{0.20};
  double slowdown_max{1.0};

  std::vector<std::string> required_sensors{
    "tof_left",
    "tof_right",
    "ultrasonic_front"};
};

struct SAVO_PERCEPTION_PUBLIC RangeFusionResult
{
  SafetyDecision decision;

  std::optional<double> front_distance_m;
  std::optional<double> side_distance_m;

  std::map<std::string, double> front_sources;
  std::map<std::string, double> side_sources;

  std::vector<std::string> stale_sensors;
  std::vector<std::string> invalid_sensors;
};

SAVO_PERCEPTION_PUBLIC std::optional<double> min_optional(
  const std::optional<double> & a,
  const std::optional<double> & b);

SAVO_PERCEPTION_PUBLIC double slowdown_from_distance(
  const std::optional<double> & distance_m,
  double stop_m,
  double slow_m,
  double slowdown_min,
  double slowdown_max);

SAVO_PERCEPTION_PUBLIC bool contains_sensor_name(
  const std::vector<std::string> & names,
  const std::string & sensor_name);

SAVO_PERCEPTION_PUBLIC std::vector<std::string> required_stale_sensors(
  const std::vector<std::string> & stale_sensors,
  const std::vector<std::string> & required_sensors);

SAVO_PERCEPTION_PUBLIC std::vector<std::string> collect_stale_sensors(
  const RangeSnapshot & snapshot,
  double stale_timeout_s);

SAVO_PERCEPTION_PUBLIC std::vector<std::string> collect_invalid_sensors(
  const RangeSnapshot & snapshot);

SAVO_PERCEPTION_PUBLIC std::map<std::string, double> front_candidates(
  const RangeSnapshot & snapshot,
  const RangeFusionConfig & config);

SAVO_PERCEPTION_PUBLIC std::map<std::string, double> side_candidates(
  const RangeSnapshot & snapshot,
  const RangeFusionConfig & config);

SAVO_PERCEPTION_PUBLIC RangeFusionResult fuse_range_snapshot(
  const RangeSnapshot & snapshot,
  const RangeFusionConfig & config);

class SAVO_PERCEPTION_PUBLIC RangeFusion
{
public:
  explicit RangeFusion(RangeFusionConfig config = RangeFusionConfig{});

  [[nodiscard]] const RangeFusionConfig & config() const;
  void set_config(RangeFusionConfig config);

  [[nodiscard]] RangeFusionResult fuse(const RangeSnapshot & snapshot) const;

private:
  RangeFusionConfig config_;
};

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__RANGE_FUSION_HPP_
