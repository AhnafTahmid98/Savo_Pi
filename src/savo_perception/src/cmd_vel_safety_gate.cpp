// File: savo_ws/src/savo_perception/src/cmd_vel_safety_gate.cpp
//
// Robot Savo — CmdVel Safety Gate (C++ / rclcpp)
// ---------------------------------------------
// Purpose:
//   Safety layer that sits between any /cmd_vel source (Nav2, teleop, etc.)
//   and the motor driver. It enforces an immediate STOP and/or a global
//   slowdown factor produced by your perception safety stack.
//
// Subscribes:
//   - /cmd_vel                (geometry_msgs::msg::Twist)
//   - /safety/stop            (std_msgs::msg::Bool)
//   - /safety/slowdown_factor (std_msgs::msg::Float32)  [0..1], optional
//
// Publishes:
//   - /cmd_vel_safe           (geometry_msgs::msg::Twist)
//
// Behavior (high level):
//   - If STOP asserted -> publish zero Twist immediately.
//   - Else apply global slowdown factor to vx, vy, omega_z (optional).
//   - Fail-safe: if stop topic is stale and fail_safe_on_stale==true -> zero Twist.
//   - Optional timer mode: re-publish last gated command at publish_hz to keep output alive.
//
// Notes:
//   - This is intentionally simple and deterministic.
//   - Directional scaling (vx/vy) can be added later when you enable it in YAML.
//
// Author: Robot Savo (savo_perception)

#include "savo_perception/safety_gate.hpp"

#include <chrono>
#include <functional>  // std::bind, std::placeholders
#include <utility>     // std::swap

namespace savo_perception
{

CmdVelSafetyGate::CmdVelSafetyGate(const rclcpp::NodeOptions & options)
: rclcpp::Node("cmd_vel_safety_gate", options)
{
  declare_and_get_params_();

  // QoS:
  // - cmd_vel is commonly RELIABLE from Nav2/teleop
  // - safety signals are "sensor-ish", BEST_EFFORT is fine and reduces backpressure
  rclcpp::QoS qos_cmd(rclcpp::KeepLast(10));
  qos_cmd.reliable();

  rclcpp::QoS qos_safety(rclcpp::KeepLast(10));
  qos_safety.best_effort();

  pub_cmd_safe_ = create_publisher<geometry_msgs::msg::Twist>(p_.cmd_out_topic, qos_cmd);

  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
    p_.cmd_in_topic,
    qos_cmd,
    std::bind(&CmdVelSafetyGate::on_cmd_, this, std::placeholders::_1));

  sub_stop_ = create_subscription<std_msgs::msg::Bool>(
    p_.stop_topic,
    qos_safety,
    std::bind(&CmdVelSafetyGate::on_stop_, this, std::placeholders::_1));

  sub_slow_ = create_subscription<std_msgs::msg::Float32>(
    p_.slowdown_topic,
    qos_safety,
    std::bind(&CmdVelSafetyGate::on_slow_, this, std::placeholders::_1));

  // Optional timer publishing:
  // Useful if downstream expects periodic commands even when upstream is quiet.
  if (p_.publish_hz > 0.0) {
    const double hz = std::max(1.0, std::min(p_.publish_hz, 200.0));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&CmdVelSafetyGate::on_timer_, this));
  }

  RCLCPP_INFO(
    get_logger(),
    "CmdVelSafetyGate online | in=%s out=%s | stop=%s slow=%s | fail_safe_on_stale=%s stale=%.2fs | "
    "use_slowdown=%s scale=[%.2f..%.2f] | publish_hz=%.1f",
    p_.cmd_in_topic.c_str(),
    p_.cmd_out_topic.c_str(),
    p_.stop_topic.c_str(),
    p_.slowdown_topic.c_str(),
    p_.fail_safe_on_stale ? "true" : "false",
    p_.stale_timeout_s,
    p_.use_slowdown ? "true" : "false",
    p_.min_scale,
    p_.max_scale,
    p_.publish_hz);
}

void CmdVelSafetyGate::declare_and_get_params_()
{
  // Topics
  declare_parameter<std::string>("cmd_in_topic", p_.cmd_in_topic);
  declare_parameter<std::string>("cmd_out_topic", p_.cmd_out_topic);
  declare_parameter<std::string>("stop_topic", p_.stop_topic);
  declare_parameter<std::string>("slowdown_topic", p_.slowdown_topic);

  // Fail-safe / staleness
  declare_parameter<bool>("fail_safe_on_stale", p_.fail_safe_on_stale);
  declare_parameter<double>("stale_timeout_s", p_.stale_timeout_s);

  // Slowdown
  declare_parameter<bool>("use_slowdown", p_.use_slowdown);
  declare_parameter<double>("min_scale", static_cast<double>(p_.min_scale));
  declare_parameter<double>("max_scale", static_cast<double>(p_.max_scale));

  // Publishing
  declare_parameter<double>("publish_hz", p_.publish_hz);

  // Load
  p_.cmd_in_topic = get_parameter("cmd_in_topic").as_string();
  p_.cmd_out_topic = get_parameter("cmd_out_topic").as_string();
  p_.stop_topic = get_parameter("stop_topic").as_string();
  p_.slowdown_topic = get_parameter("slowdown_topic").as_string();

  p_.fail_safe_on_stale = get_parameter("fail_safe_on_stale").as_bool();
  p_.stale_timeout_s = get_parameter("stale_timeout_s").as_double();
  p_.stale_timeout_s = std::max(0.05, std::min(p_.stale_timeout_s, 5.0));

  p_.use_slowdown = get_parameter("use_slowdown").as_bool();

  const double min_s = get_parameter("min_scale").as_double();
  const double max_s = get_parameter("max_scale").as_double();
  p_.min_scale = static_cast<float>(clampf(static_cast<float>(min_s), 0.0f, 1.0f));
  p_.max_scale = static_cast<float>(clampf(static_cast<float>(max_s), 0.0f, 1.0f));
  if (p_.min_scale > p_.max_scale) {
    std::swap(p_.min_scale, p_.max_scale);
  }

  p_.publish_hz = get_parameter("publish_hz").as_double();
  if (p_.publish_hz < 0.0) {
    p_.publish_hz = 0.0;
  } else if (p_.publish_hz > 200.0) {
    p_.publish_hz = 200.0;
  }
}

void CmdVelSafetyGate::on_cmd_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  last_cmd_ = *msg;
  have_cmd_ = true;

  // If timer is disabled, publish immediately on every cmd input.
  if (!timer_) {
    pub_cmd_safe_->publish(apply_gate_(last_cmd_));
  }
}

void CmdVelSafetyGate::on_stop_(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  stop_ = msg->data;
  last_stop_rx_s_ = now().seconds();

  // Immediate hard stop publish (even if timer mode is enabled).
  if (stop_) {
    pub_cmd_safe_->publish(geometry_msgs::msg::Twist{});
  }
}

void CmdVelSafetyGate::on_slow_(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const float v = static_cast<float>(msg->data);
  scale_ = clampf(v, p_.min_scale, p_.max_scale);
  last_scale_rx_s_ = now().seconds();
}

void CmdVelSafetyGate::on_timer_()
{
  if (!have_cmd_) {
    return;  // nothing seen yet
  }
  pub_cmd_safe_->publish(apply_gate_(last_cmd_));
}

bool CmdVelSafetyGate::stop_is_stale_(double now_s) const
{
  if (last_stop_rx_s_ <= 0.0) {
    return true;  // never received stop yet
  }
  return (now_s - last_stop_rx_s_) > p_.stale_timeout_s;
}

geometry_msgs::msg::Twist CmdVelSafetyGate::apply_gate_(const geometry_msgs::msg::Twist & in) const
{
  // Start with the incoming command
  geometry_msgs::msg::Twist out = in;

  const double t_now_s = now().seconds();

  // Fail-safe: if stop signal is stale -> zero output
  if (p_.fail_safe_on_stale && stop_is_stale_(t_now_s)) {
    return geometry_msgs::msg::Twist{};
  }

  // Hard stop wins
  if (stop_) {
    return geometry_msgs::msg::Twist{};
  }

  // Optional global slowdown
  if (p_.use_slowdown) {
    const float s = clampf(scale_, p_.min_scale, p_.max_scale);
    out.linear.x *= s;
    out.linear.y *= s;
    out.angular.z *= s;
  }

  // Lock unused axes to zero (Twist convention)
  out.linear.z = 0.0;
  out.angular.x = 0.0;
  out.angular.y = 0.0;

  return out;
}

}  // namespace savo_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_perception::CmdVelSafetyGate>());
  rclcpp::shutdown();
  return 0;
}
