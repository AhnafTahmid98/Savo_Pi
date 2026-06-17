#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "savo_lidar/scan_types.hpp"
#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

class SAVO_LIDAR_PUBLIC ScanPublisher
{
public:
  ScanPublisher(
    rclcpp::Node * node,
    const std::string & topic_name,
    const rclcpp::QoS & qos);

  void publish(const LidarScan & scan);

  const std::string & topic_name() const noexcept;
  std::uint64_t publish_count() const noexcept;

private:
  sensor_msgs::msg::LaserScan to_message(const LidarScan & scan) const;

  rclcpp::Node * node_{nullptr};
  std::string topic_name_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::uint64_t publish_count_{0};
};

SAVO_LIDAR_PUBLIC sensor_msgs::msg::LaserScan to_laser_scan_msg(
  const LidarScan & scan,
  const rclcpp::Time & stamp);

}  // namespace savo_lidar
