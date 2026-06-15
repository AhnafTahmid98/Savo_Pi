#include "savo_lidar/scan_publisher.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

namespace savo_lidar
{

ScanPublisher::ScanPublisher(
  rclcpp::Node * node,
  const std::string & topic_name,
  const rclcpp::QoS & qos)
: node_(node),
  topic_name_(topic_name)
{
  if (node_ == nullptr) {
    throw std::invalid_argument("ScanPublisher node cannot be null");
  }

  if (topic_name_.empty()) {
    throw std::invalid_argument("ScanPublisher topic_name cannot be empty");
  }

  publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    topic_name_,
    qos);
}

void ScanPublisher::publish(const LidarScan & scan)
{
  if (!publisher_) {
    throw std::runtime_error("LaserScan publisher is not initialized");
  }

  auto msg = to_message(scan);
  publisher_->publish(std::move(msg));
  ++publish_count_;
}

const std::string & ScanPublisher::topic_name() const noexcept
{
  return topic_name_;
}

std::uint64_t ScanPublisher::publish_count() const noexcept
{
  return publish_count_;
}

sensor_msgs::msg::LaserScan ScanPublisher::to_message(const LidarScan & scan) const
{
  return to_laser_scan_msg(scan, node_->now());
}

sensor_msgs::msg::LaserScan to_laser_scan_msg(
  const LidarScan & scan,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::LaserScan msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = scan.frame_id;

  msg.angle_min = static_cast<float>(scan.angle_min_rad);
  msg.angle_max = static_cast<float>(scan.angle_max_rad);
  msg.angle_increment = static_cast<float>(scan.angle_increment_rad);

  msg.time_increment = static_cast<float>(scan.time_increment_s);
  msg.scan_time = static_cast<float>(scan.scan_time_s);

  msg.range_min = scan.range_min_m;
  msg.range_max = scan.range_max_m;

  msg.ranges = scan.ranges_m;

  if (scan.intensities.size() == scan.ranges_m.size()) {
    msg.intensities = scan.intensities;
  } else {
    msg.intensities.assign(scan.ranges_m.size(), 0.0F);
  }

  return msg;
}

}  // namespace savo_lidar