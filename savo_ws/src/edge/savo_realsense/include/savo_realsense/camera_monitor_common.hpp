#pragma once

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
  double expected_color_hz{20.0};
  double expected_depth_hz{20.0};
  double expected_camera_info_hz{20.0};
  double expected_pointcloud_hz{8.0};
};

struct StreamStatus
{
  std::string topic;
  bool seen{false};
  bool stale{true};
  double rate_hz{0.0};
  double expected_hz{0.0};
  double last_age_s{0.0};

  bool ok() const;
};

class RateTracker
{
public:
  explicit RateTracker(std::size_t window_size = 64);

  void tick(const rclcpp::Time & now);
  bool seen() const;
  double rate_hz() const;
  double last_age_s(const rclcpp::Time & now) const;

private:
  bool seen_{false};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  std::deque<double> intervals_;
  std::size_t window_size_{64};
};

StreamStatus build_stream_status(
  const std::string & topic,
  const RateTracker & tracker,
  const rclcpp::Time & now,
  double expected_hz,
  double stale_timeout_s);

diagnostic_msgs::msg::DiagnosticStatus make_stream_diagnostic(
  const std::string & name,
  const StreamStatus & status);

diagnostic_msgs::msg::DiagnosticArray make_diagnostic_array(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & statuses,
  const rclcpp::Time & stamp);

std::string bool_text(bool value);
std::string number_text(double value);

}  // namespace savo_realsense
