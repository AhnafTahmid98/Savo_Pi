#pragma once

#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace savo_vo
{

class VORepublisherNode final : public rclcpp::Node
{
public:
  explicit VORepublisherNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Odometry = nav_msgs::msg::Odometry;

  static rclcpp::QoS odometry_qos();

  void declare_parameters();
  void load_parameters();
  void create_publishers();
  void create_subscribers();

  void on_odom(const Odometry::SharedPtr msg);

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
};

}  // namespace savo_vo
