#include "savo_vo/timestamp_sync.hpp"

#include <algorithm>
#include <cmath>

namespace savo_vo
{

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

double stamp_delta_seconds(
  const builtin_interfaces::msg::Time & first,
  const builtin_interfaces::msg::Time & second)
{
  return std::abs(stamp_to_seconds(first) - stamp_to_seconds(second));
}

double max_stamp_delta_seconds(
  const builtin_interfaces::msg::Time & color_stamp,
  const builtin_interfaces::msg::Time & depth_stamp,
  const builtin_interfaces::msg::Time & camera_info_stamp)
{
  const double color_depth_delta = stamp_delta_seconds(color_stamp, depth_stamp);
  const double color_info_delta = stamp_delta_seconds(color_stamp, camera_info_stamp);
  const double depth_info_delta = stamp_delta_seconds(depth_stamp, camera_info_stamp);

  return std::max({
    color_depth_delta,
    color_info_delta,
    depth_info_delta,
  });
}

TimestampSyncStatus evaluate_timestamp_sync(
  const builtin_interfaces::msg::Time & color_stamp,
  const builtin_interfaces::msg::Time & depth_stamp,
  const builtin_interfaces::msg::Time & camera_info_stamp,
  const double max_allowed_delta_s)
{
  TimestampSyncStatus status;
  status.color_stamp_s = stamp_to_seconds(color_stamp);
  status.depth_stamp_s = stamp_to_seconds(depth_stamp);
  status.camera_info_stamp_s = stamp_to_seconds(camera_info_stamp);
  status.max_delta_s = max_stamp_delta_seconds(
    color_stamp,
    depth_stamp,
    camera_info_stamp);

  const double safe_limit_s = std::max(0.0, max_allowed_delta_s);
  status.synchronized = status.max_delta_s <= safe_limit_s;

  return status;
}

}  // namespace savo_vo
