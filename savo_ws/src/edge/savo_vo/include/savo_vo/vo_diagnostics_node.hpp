#pragma once

#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace savo_vo
{

class VODiagnosticsNode final : public rclcpp::Node
{
public:
  explicit VODiagnosticsNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using String = std_msgs::msg::String;

  static rclcpp::QoS diagnostics_qos();
  static rclcpp::QoS status_qos();

  void declare_parameters();
  void load_parameters();
  void create_publishers();
  void create_subscribers();
  void create_timers();

  void on_status(const String::SharedPtr msg);
  void on_health(const String::SharedPtr msg);

  void publish_diagnostics();
  DiagnosticStatus build_diagnostic_status() const;

  std::string status_topic_;
  std::string health_topic_;
  std::string diagnostics_topic_;

  double diagnostics_rate_hz_{2.0};

  std::string last_status_{"waiting for visual odometry"};
  std::string last_health_{"waiting"};

  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<String>::SharedPtr status_sub_;
  rclcpp::Subscription<String>::SharedPtr health_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_vo
