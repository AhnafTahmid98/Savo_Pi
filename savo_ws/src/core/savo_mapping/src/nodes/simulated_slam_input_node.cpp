#include "savo_mapping/parameter_utils.hpp"
#include "savo_mapping/qos_profiles.hpp"
#include "savo_mapping/simulated_scan_model.hpp"
#include "savo_mapping/topic_names.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace savo_mapping
{

namespace
{

std::string require_non_empty(
  const std::string & parameter_name,
  std::string value)
{
  if (value.empty()) {
    throw std::invalid_argument(
            "parameter '" + parameter_name +
            "' must not be empty");
  }

  return value;
}

std::chrono::nanoseconds rate_to_period(
  double rate_hz)
{
  return std::chrono::duration_cast<
    std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz));
}

}  // namespace

class SimulatedSlamInputNode final : public rclcpp::Node
{
public:
  SimulatedSlamInputNode()
  : Node("simulated_slam_input_node")
  {
    declare_parameters();
    configure_scan_model();
    create_interfaces();
    publish_static_transform();

    publish_timer_ = create_wall_timer(
      rate_to_period(publish_rate_hz_),
      std::bind(
        &SimulatedSlamInputNode::publish_cycle,
        this));

    RCLCPP_WARN(
      get_logger(),
      "SIMULATION FIXTURE ACTIVE: publishing scan=%s "
      "odom=%s frames=%s->%s->%s",
      scan_topic_.c_str(),
      odom_topic_.c_str(),
      odom_frame_.c_str(),
      base_frame_.c_str(),
      lidar_frame_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "simulated room %.2fm x %.2fm, samples=%zu, rate=%.1fHz",
      scan_config_.half_width_m * 2.0,
      scan_config_.half_height_m * 2.0,
      scan_config_.sample_count,
      publish_rate_hz_);
  }

private:
  void declare_parameters()
  {
    publish_rate_hz_ =
      params::require_positive_parameter(
      "simulation.publish_rate_hz",
      params::declare_or_get<double>(
        *this,
        "simulation.publish_rate_hz",
        10.0));

    scan_config_.half_width_m =
      params::require_positive_parameter(
      "simulation.room_half_width_m",
      params::declare_or_get<double>(
        *this,
        "simulation.room_half_width_m",
        2.50));

    scan_config_.half_height_m =
      params::require_positive_parameter(
      "simulation.room_half_height_m",
      params::declare_or_get<double>(
        *this,
        "simulation.room_half_height_m",
        1.75));

    const std::int64_t sample_count =
      params::require_integer_parameter_in_closed_range(
      "simulation.sample_count",
      params::declare_or_get<std::int64_t>(
        *this,
        "simulation.sample_count",
        360),
      36,
      1440);

    scan_config_.sample_count =
      static_cast<std::size_t>(sample_count);

    scan_config_.range_min_m =
      params::require_positive_parameter(
      "simulation.range_min_m",
      params::declare_or_get<double>(
        *this,
        "simulation.range_min_m",
        0.15));

    scan_config_.range_max_m =
      params::require_positive_parameter(
      "simulation.range_max_m",
      params::declare_or_get<double>(
        *this,
        "simulation.range_max_m",
        12.0));

    scan_topic_ = require_non_empty(
      "topics.scan",
      params::declare_or_get<std::string>(
        *this,
        "topics.scan",
        std::string{topics::SCAN}));

    odom_topic_ = require_non_empty(
      "topics.odom",
      params::declare_or_get<std::string>(
        *this,
        "topics.odom",
        std::string{topics::ODOM_FILTERED}));

    odom_frame_ = require_non_empty(
      "frames.odom",
      params::declare_or_get<std::string>(
        *this,
        "frames.odom",
        std::string{"odom"}));

    base_frame_ = require_non_empty(
      "frames.base",
      params::declare_or_get<std::string>(
        *this,
        "frames.base",
        std::string{"base_link"}));

    lidar_frame_ = require_non_empty(
      "frames.lidar",
      params::declare_or_get<std::string>(
        *this,
        "frames.lidar",
        std::string{"laser_frame"}));
  }

  void configure_scan_model()
  {
    const std::string error =
      simulation::validate_simulated_room_scan_config(
      scan_config_);

    if (!error.empty()) {
      throw std::invalid_argument(
              "invalid simulated scan model: " + error);
    }

    simulated_ranges_ =
      simulation::generate_rectangular_room_scan(
      scan_config_);
  }

  void create_interfaces()
  {
    scan_publisher_ =
      create_publisher<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      qos::scan_qos());

    odom_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile());

    transform_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(
      *this);

    static_transform_broadcaster_ =
      std::make_unique<
      tf2_ros::StaticTransformBroadcaster>(*this);
  }

  void publish_static_transform()
  {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = now();
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = lidar_frame_;

    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    static_transform_broadcaster_->sendTransform(
      transform);
  }

  void publish_cycle()
  {
    const rclcpp::Time stamp = now();

    publish_odom_transform(stamp);
    publish_odometry(stamp);
    publish_scan(stamp);

    ++sequence_;
  }

  void publish_odom_transform(
    const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;

    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    transform_broadcaster_->sendTransform(transform);
  }

  void publish_odometry(
    const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odometry;

    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;

    odometry.pose.pose.position.x = 0.0;
    odometry.pose.pose.position.y = 0.0;
    odometry.pose.pose.position.z = 0.0;

    odometry.pose.pose.orientation.x = 0.0;
    odometry.pose.pose.orientation.y = 0.0;
    odometry.pose.pose.orientation.z = 0.0;
    odometry.pose.pose.orientation.w = 1.0;

    odometry.pose.covariance[0] = 1.0e-6;
    odometry.pose.covariance[7] = 1.0e-6;
    odometry.pose.covariance[35] = 1.0e-6;

    odometry.twist.covariance[0] = 1.0e-6;
    odometry.twist.covariance[7] = 1.0e-6;
    odometry.twist.covariance[35] = 1.0e-6;

    odom_publisher_->publish(odometry);
  }

  void publish_scan(
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = stamp;
    scan.header.frame_id = lidar_frame_;

    scan.angle_min = static_cast<float>(
      simulation::scan_angle_min_rad());

    scan.angle_max = static_cast<float>(
      simulation::scan_angle_max_rad(
        scan_config_.sample_count));

    scan.angle_increment = static_cast<float>(
      simulation::scan_angle_increment_rad(
        scan_config_.sample_count));

    scan.scan_time = static_cast<float>(
      1.0 / publish_rate_hz_);

    scan.time_increment =
      scan.scan_time /
      static_cast<float>(scan_config_.sample_count);

    scan.range_min = static_cast<float>(
      scan_config_.range_min_m);

    scan.range_max = static_cast<float>(
      scan_config_.range_max_m);

    scan.ranges = simulated_ranges_;

    scan_publisher_->publish(scan);
  }

  double publish_rate_hz_{10.0};

  simulation::SimulatedRoomScanConfig scan_config_;
  std::vector<float> simulated_ranges_;

  std::string scan_topic_;
  std::string odom_topic_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;

  std::uint64_t sequence_{0};

  rclcpp::Publisher<
    sensor_msgs::msg::LaserScan>::SharedPtr
    scan_publisher_;

  rclcpp::Publisher<
    nav_msgs::msg::Odometry>::SharedPtr
    odom_publisher_;

  std::unique_ptr<
    tf2_ros::TransformBroadcaster>
    transform_broadcaster_;

  std::unique_ptr<
    tf2_ros::StaticTransformBroadcaster>
    static_transform_broadcaster_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        savo_mapping::SimulatedSlamInputNode>());
  } catch (const std::exception & exception) {
    std::cerr
      << "simulated_slam_input_node failed: "
      << exception.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
