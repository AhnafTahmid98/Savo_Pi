#pragma once

#include "builtin_interfaces/msg/time.hpp"

#include "savo_vo/vo_types.hpp"

namespace savo_vo
{

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp);

double stamp_delta_seconds(
  const builtin_interfaces::msg::Time & first,
  const builtin_interfaces::msg::Time & second);

double max_stamp_delta_seconds(
  const builtin_interfaces::msg::Time & color_stamp,
  const builtin_interfaces::msg::Time & depth_stamp,
  const builtin_interfaces::msg::Time & camera_info_stamp);

TimestampSyncStatus evaluate_timestamp_sync(
  const builtin_interfaces::msg::Time & color_stamp,
  const builtin_interfaces::msg::Time & depth_stamp,
  const builtin_interfaces::msg::Time & camera_info_stamp,
  double max_allowed_delta_s);

}  // namespace savo_vo
