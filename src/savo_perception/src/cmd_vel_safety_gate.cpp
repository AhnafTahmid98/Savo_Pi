// File: savo_ws/src/savo_perception/src/cmd_vel_safety_gate.cpp
//
// Robot Savo — CmdVel Safety Gate (C++ / rclcpp)
//
// Subscribes:
//   - /cmd_vel                  (geometry_msgs::msg::Twist)
//   - /safety/stop              (std_msgs::msg::Bool)
//   - /safety/slowdown_factor   (std_msgs::msg::Float32)  [0..1], optional
//
// Publishes:
//   - /cmd_vel_safe             (geometry_msgs::msg::Twist)
//
// Behavior:
//   - If stop==true -> publish zero Twist
//   - Else apply global slowdown_factor to vx, vy, omega (optional)
//   - Fail-safe: if stop topic becomes stale and fail_safe_on_stale==true -> zero Twist
//
// Notes:
//   - This node should sit between Nav2/teleop and the motor driver.
//   - Configure your motor driver or cmd_vel mux to consume /cmd_vel_safe.
//

#include "savo_perception/safety_gate.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace savo_perception
{

CmdVelSafetyGate::CmdVelSafetyGate(const rclcpp::NodeOptions & options)
: rclcpp::Node("cmd_vel_safety_gate", options)
{
  declare_and_get_params_();

  // QoS: cmd_vel is typically reliable; safety topics are sensor-ish (best-effort ok)
  rclcpp::QoS qos_cmd(rclcpp::KeepLast(10));
  qos_cmd.reliable();

  rclcpp::QoS qos_sensor(rclcpp::KeepLast(10));
  qos_sensor.best_effort();

  pub_cmd_safe_ = this->create_publisher<geometry_msgs::msg::Twist>(p_.cmd_out_topic, qos_cmd);

  sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
    p_.cmd_in_topic, qos_cmd,
    std::bind(&CmdVelSafetyGate::on_cmd_, this, std::placeholders::_1));

  sub_stop_ = this->create_subscription<std_msgs::msg::Bool>(
    p_.stop_topic, qos_sensor,
    std::bind(&CmdVelSafetyGate::on_stop_, this, std::placeholders::_1));

  sub_slow_ = this->create_subscription<std_msgs::msg::Float32>(
    p_.slowdown_topic, qos_sensor,
    std::bind(&CmdVelSafetyGate::on_slow_, this, std::placeholders::_1));

  if (p_.publish_hz > 0.0) {
    const double hz = std::max(1.0, std::min(p_.publish_hz, 200.0));
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&CmdVelSafetyGate::on_timer_, this));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "CmdVelSafetyGate started. in=%s out=%s stop=%s slow=%s fail_safe_on_stale=%s stale=%.2fs use_slowdown=%s "
    "scale=[%.2f..%.2f] publish_hz=%.1f",
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
  this->declare_parameter<std::string>("cmd_in_topic", p_.cmd_in_topic);
  this->declare_parameter<std::string>("cmd_out_topic", p_.cmd_out_topic);
  this->declare_parameter<std::string>("stop_topic", p_.stop_topic);
  this->declare_parameter<std::string>("slowdown_topic", p_.slowdown_topic);

  // Fail-safe / staleness
  this->declare_parameter<bool>("fail_safe_on_stale", p_.fail_safe_on_stale);
  this->declare_parameter<double>("stale_timeout_s", p_.stale_timeout_s);

  // Slowdown
  this->declare_parameter<bool>("use_slowdown", p_.use_slowdown);
  this->declare_parameter<double>("min_scale", static_cast<double>(p_.min_scale));
  this->declare_parameter<double>("max_scale", static_cast<double>(p_.max_scale));

  // Publishing
  this->declare_parameter<double>("publish_hz", p_.publish_hz);

  // Load
  p_.cmd_in_topic = this->get_parameter("cmd_in_topic").as_string();
  p_.cmd_out_topic = this->get_parameter("cmd_out_topic").as_string();
  p_.stop_topic = this->get_parameter("stop_topic").as_string();
  p_.slowdown_topic = this->get_parameter("slowdown_topic").as_string();

  p_.fail_safe_on_stale = this->get_parameter("fail_safe_on_stale").as_bool();
  p_.stale_timeout_s = this->get_parameter("stale_timeout_s").as_double();
  p_.stale_timeout_s = std::max(0.05, std::min(p_.stale_timeout_s, 5.0));

  p_.use_slowdown = this->get_parameter("use_slowdown").as_bool();

  const double min_s = this->get_parameter("min_scale").as_double();
  const double max_s = this->get_parameter("max_scale").as_double();
  p_.min_scale = static_cast<float>(clampf(static_cast<float>(min_s), 0.0f, 1.0f));
  p_.max_scale = static_cast<float>(clampf(static_cast<float>(max_s), 0.0f, 1.0f));
  if (p_.min_scale > p_.max_scale) {
    std::swap(p_.min_scale, p_.max_scale);
  }

  p_.publish_hz = this->get_parameter("publish_hz").as_double();
  if (p_.publish_hz < 0.0) {
    p_.publish_hz = 0.0;
  }
  if (p_.publish_hz > 200.0) {
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

  // If we don't have a fixed timer loop, publish immediately on cmd input.
  if (!timer_) {
    const auto out = apply_gate_(last_cmd_);
    pub_cmd_safe_->publish(out);
  }
}

void CmdVelSafetyGate::on_stop_(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  stop_ = msg->data;
  last_stop_rx_s_ = this->get_clock()->now().seconds();

  // If stop asserted, publish zero immediately (even if timer loop is enabled).
  if (stop_) {
    geometry_msgs::msg::Twist z{};
    pub_cmd_safe_->publish(z);
  }
}

void CmdVelSafetyGate::on_slow_(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  // Clamp to [min_scale, max_scale]
  const float v = static_cast<float>(msg->data);
  const float c = clampf(v, p_.min_scale, p_.max_scale);
  scale_ = c;
  last_scale_rx_s_ = this->get_clock()->now().seconds();
}

void CmdVelSafetyGate::on_timer_()
{
  if (!have_cmd_) {
    return;  // nothing to publish yet
  }
  const auto out = apply_gate_(last_cmd_);
  pub_cmd_safe_->publish(out);
}

bool CmdVelSafetyGate::stop_is_stale_(double now_s) const
{
  if (last_stop_rx_s_ <= 0.0) {
    return true;  // never received stop yet -> treat as stale
  }
  return (now_s - last_stop_rx_s_) > p_.stale_timeout_s;
}

geometry_msgs::msg::Twist CmdVelSafetyGate::apply_gate_(const geometry_msgs::msg::Twist & in) const
{
  geometry_msgs::msg::Twist out = in;

  const double now_s = this->get_clock()->now().seconds();

  // Fail-safe: if stop topic is stale -> stop
  if (p_.fail_safe_on_stale && stop_is_stale_(now_s)) {
    out.linear.x = 0.0;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;
    out.angular.z = 0.0;
    return out;
  }

  // Hard stop wins
  if (stop_) {
    out.linear.x = 0.0;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;
    out.angular.z = 0.0;
    return out;
  }

  // Optional global slowdown
  if (p_.use_slowdown) {
    const float s = clampf(scale_, p_.min_scale, p_.max_scale);
    out.linear.x *= s;
    out.linear.y *= s;
    out.angular.z *= s;
  }

  // Keep other fields at zero for safety (Twist convention)
  out.linear.z = 0.0;
  out.angular.x = 0.0;
  out.angular.y = 0.0;

  return out;
}

}  // namespace savo_perception

// Standard ROS2 component entrypoint is optional; we keep it as a normal node executable.
// If you later want to load as a component, add RCLCPP_COMPONENTS_REGISTER_NODE.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_perception::CmdVelSafetyGate>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
