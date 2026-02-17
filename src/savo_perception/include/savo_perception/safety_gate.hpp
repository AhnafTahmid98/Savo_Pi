// File: savo_ws/src/savo_perception/include/savo_perception/safety_gate.hpp
#pragma once

#include <string>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace savo_perception
{

// Clamp helper
inline float clampf(float v, float lo, float hi)
{
  return std::max(lo, std::min(v, hi));
}

/**
 * @brief CmdVel Safety Gate
 *
 * Subscribes:
 *   - /cmd_vel                  (geometry_msgs/Twist)
 *   - /safety/stop              (std_msgs/Bool)
 *   - /safety/slowdown_factor   (std_msgs/Float32)  [0..1], optional
 *
 * Publishes:
 *   - /cmd_vel_safe             (geometry_msgs/Twist)
 *
 * Behavior:
 *   - If stop==true -> publish zero Twist
 *   - Else apply global slowdown_factor to vx, vy, omega (optional per-axis can be added later)
 *   - Fail-safe: if stop topic becomes stale and fail_safe_on_stale==true -> zero Twist
 */
class CmdVelSafetyGate : public rclcpp::Node
{
public:
  struct Params
  {
    // Topics
    std::string cmd_in_topic{"/cmd_vel"};
    std::string cmd_out_topic{"/cmd_vel_safe"};
    std::string stop_topic{"/safety/stop"};
    std::string slowdown_topic{"/safety/slowdown_factor"};

    // Staleness / fail-safe
    bool fail_safe_on_stale{true};
    double stale_timeout_s{0.30};

    // Slowdown behavior
    bool use_slowdown{true};
    float min_scale{0.20f};   // floor for slowdown_factor when used (safety_stop_node already enforces, but keep safe)
    float max_scale{1.00f};

    // Output rate limit (optional): if 0 -> publish only on cmd input callback
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

  // Latest inputs
  geometry_msgs::msg::Twist last_cmd_{};
  bool have_cmd_{false};

  bool stop_{false};
  double last_stop_rx_s_{0.0};

  float scale_{1.0f};
  double last_scale_rx_s_{0.0};

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_slow_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_safe_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_perception
