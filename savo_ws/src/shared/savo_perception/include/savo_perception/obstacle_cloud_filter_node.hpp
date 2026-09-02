// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include "savo_perception/obstacle_cloud_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstddef>
#include <memory>
#include <string>

namespace savo_perception
{

class ObstacleCloudFilterNode final
  : public rclcpp::Node
{
public:
  ObstacleCloudFilterNode();

private:
  void declare_and_read_parameters();
  std::string validate_parameters() const;
  void handle_cloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  bool validate_cloud_layout(
    const sensor_msgs::msg::PointCloud2 & message,
    std::string & reason) const;
  void reject_cloud(
    const std::string & state,
    const std::string & reason,
    bool transform_failure);
  void publish_health_and_status();
  void publish_heartbeat();
  std::string make_status_json() const;

  ObstacleCloudFilterConfig filter_config_;

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  std::string health_topic_;
  std::string status_topic_;
  std::string heartbeat_topic_;

  double transform_timeout_s_{0.10};
  double max_processing_hz_{10.0};
  double stale_timeout_s_{0.75};
  double status_publish_hz_{2.0};
  double heartbeat_hz_{1.0};
  bool startup_fail_is_fatal_{false};

  bool configuration_valid_{false};
  bool healthy_{false};
  bool have_valid_input_{false};

  std::string state_{"starting"};
  std::string reason_{"waiting_for_valid_cloud"};
  std::string input_frame_;

  ObstacleCloudFilterStats last_stats_;
  std::size_t transform_failures_{0U};
  std::size_t malformed_clouds_{0U};
  std::size_t clouds_received_{0U};
  std::size_t clouds_published_{0U};
  std::size_t clouds_rate_limited_{0U};
  std::size_t heartbeat_counter_{0U};

  rclcpp::Time last_valid_input_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>
  tf_listener_;

  std::unique_ptr<ObstacleCloudProcessingGate>
  processing_gate_;

  std::unique_ptr<ObstacleCloudFilterAccumulator>
  filter_accumulator_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_subscription_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_publisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    health_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    status_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    heartbeat_publisher_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace savo_perception
