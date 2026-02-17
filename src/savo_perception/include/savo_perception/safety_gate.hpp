// File: Savo_Pi/src/savo_perception/include/savo_perception/safety_gate.hpp
#pragma once

#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace savo_perception
{

inline float clampf(float v, float lo, float hi)
{
  return std::max(lo, std::min(v, hi));
}

class CmdVelSafetyGate : public rclcpp::Node
{
public:
  struct Params
  {
    std::string cmd_in_topic{"/cmd_vel"};
    std::string cmd_out_topic{"/cmd_vel_safe"};
    std::string stop_topic{"/safety/stop"};
    std::string slowdown_topic{"/safety/slowdown_factor"};

    bool fail_safe_on_stale{true};
    double stale_timeout_s{0.30};

    bool use_slowdown{true};
    float min_scale{0.20f};
    float max_scale{1.00f};

    double publish_hz{0.0};
  };

  explicit CmdVelSafetyGate(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_and_get_params_();

  void on_cmd_(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_stop_(const std_msgs::msg::Bool::SharedPtr msg);
  void on_slow_(const std_msgs::msg::Float32::SharedPtr msg);
  void on_timer_();

  geometry_msgs::msg::Twist apply_gate_(const geometry_msgs::msg::Twist & in) const;
  bool stop_is_stale_(double now_s) const;

private:
  Params p_;

  geometry_msgs::msg::Twist last_cmd_{};
  bool have_cmd_{false};

  bool stop_{false};
  double last_stop_rx_s_{0.0};

  float scale_{1.0f};
  double last_scale_rx_s_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_slow_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_safe_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_perception
