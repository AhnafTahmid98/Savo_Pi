#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace savo_realsense
{

struct StreamMonitorParams
{
  double stale_timeout_s{0.75};
  double expected_color_hz{15.0};
  double expected_depth_hz{15.0};
  double expected_aligned_depth_hz{15.0};
  double expected_camera_info_hz{15.0};
  double expected_pointcloud_hz{8.0};
  double camera_minimum_hz{8.0};
  double camera_good_hz{12.0};
  double camera_excellent_hz{14.0};
  double pointcloud_minimum_hz{3.0};
  double pointcloud_good_hz{5.0};
  double pointcloud_excellent_hz{7.0};
};

struct StreamStatus
{
  std::string topic;
  bool seen{false};
  bool stale{true};
  double rate_hz{0.0};
  double expected_hz{0.0};
  double last_age_s{0.0};
  double minimum_hz{0.0};
  double good_hz{0.0};
  double excellent_hz{0.0};
  std::string rate_quality{"BELOW_MINIMUM"};

  bool ok() const;
};

class RateTracker
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit RateTracker(std::size_t window_size = 64);

  void tick(TimePoint now);
  bool seen() const;
  double rate_hz() const;
  double last_age_s(TimePoint now) const;

private:
  bool seen_{false};
  TimePoint last_time_{};
  std::deque<double> intervals_;
  std::size_t window_size_{64};
};

StreamStatus build_stream_status(
  const std::string & topic,
  const RateTracker & tracker,
  RateTracker::TimePoint now,
  double expected_hz,
  double stale_timeout_s,
  double minimum_hz,
  double good_hz,
  double excellent_hz);

std::string classify_rate_quality(
  double rate_hz,
  double minimum_hz,
  double good_hz,
  double excellent_hz);

diagnostic_msgs::msg::DiagnosticStatus make_stream_diagnostic(
  const std::string & name,
  const StreamStatus & status);

diagnostic_msgs::msg::DiagnosticArray make_diagnostic_array(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & statuses,
  const rclcpp::Time & stamp);

std::string bool_text(bool value);
std::string number_text(double value);

}  // namespace savo_realsense
