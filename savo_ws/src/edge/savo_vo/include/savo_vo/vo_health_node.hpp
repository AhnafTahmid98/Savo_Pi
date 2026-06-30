#pragma once

#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace savo_vo
{

class VOHealthNode final : public rclcpp::Node
{
public:
  explicit VOHealthNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Odometry = nav_msgs::msg::Odometry;
  using String = std_msgs::msg::String;

  static rclcpp::QoS odometry_qos();
  static rclcpp::QoS status_qos();

  void declare_parameters();
  void load_parameters();
  void create_publishers();
  void create_subscribers();
  void create_timers();

  void on_odom(const Odometry::SharedPtr msg);
  void on_status(const String::SharedPtr msg);

  void publish_health();
  std::string build_health_message(double now_s) const;

  std::string odom_topic_;
  std::string status_topic_;
  std::string health_topic_;

  double stale_timeout_s_{0.50};
  double last_odom_time_s_{0.0};

  bool has_odom_{false};
  std::string last_status_{"waiting for visual odometry"};

  rclcpp::Publisher<String>::SharedPtr health_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<String>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_vo
