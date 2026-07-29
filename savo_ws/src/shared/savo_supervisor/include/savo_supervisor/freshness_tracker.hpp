// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

namespace savo_supervisor
{

struct FreshnessSnapshot
{
  bool disabled = false;
  bool received = false;
  bool stale = false;
  bool malformed = false;
  bool timestamp_fault = false;
  bool time_regression = false;
  bool valid = false;

  double age_s = 0.0;
  double timeout_s = 0.0;

  std::string detail;

  std::size_t malformed_count = 0;
  std::size_t recovery_count = 0;

  rclcpp::Time last_receive_time =
    rclcpp::Time(0, 0, RCL_ROS_TIME);

  builtin_interfaces::msg::Time last_header_stamp{};
};

class FreshnessTracker
{
public:
  FreshnessTracker() = default;

  void mark_disabled();

  void observe_message(
    const rclcpp::Time & receive_time,
    const std::optional<builtin_interfaces::msg::Time> & header_stamp,
    bool malformed,
    const std::string & detail);

  FreshnessSnapshot snapshot(
    const rclcpp::Time & now,
    double timeout_s) const;

  bool received() const;
  std::size_t malformed_count() const;
  std::size_t recovery_count() const;

private:
  bool disabled_ = false;
  bool received_ = false;
  bool malformed_ = false;
  bool timestamp_fault_ = false;

  mutable bool time_regression_ = false;
  mutable bool stale_active_ = false;

  std::size_t malformed_count_ = 0;
  mutable std::size_t recovery_count_ = 0;

  rclcpp::Time last_receive_time_ =
    rclcpp::Time(0, 0, RCL_ROS_TIME);

  int64_t last_header_stamp_ns_ = -1;
  builtin_interfaces::msg::Time last_header_stamp_{};

  std::string detail_;
};

}  // namespace savo_supervisor
