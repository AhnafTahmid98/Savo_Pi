#include "savo_base/state/state_summary.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

namespace savo_base
{
namespace
{
double monotonic_now()
{
  // Linux steady_clock and Python time.monotonic both use CLOCK_MONOTONIC.
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class BaseStatePublisherNode : public rclcpp::Node
{
public:
  BaseStatePublisherNode() : Node("base_state_publisher_node")
  {
    summary::Parameters p;
    p.robot_name = declare_parameter("robot_name", p.robot_name);
    p.cmd_topic = declare_parameter("cmd_topic", p.cmd_topic);
    p.safety_stop_topic = declare_parameter("safety_stop_topic", p.safety_stop_topic);
    p.slowdown_topic = declare_parameter("slowdown_topic", p.slowdown_topic);
    p.watchdog_topic = declare_parameter("watchdog_topic", p.watchdog_topic);
    p.base_state_topic = declare_parameter("base_state_topic", p.base_state_topic);
    p.motor_board_status_topic = declare_parameter("motor_board_status_topic", p.motor_board_status_topic);
    p.subscribe_cmd = declare_parameter("subscribe_cmd", p.subscribe_cmd);
    p.subscribe_safety_stop = declare_parameter("subscribe_safety_stop", p.subscribe_safety_stop);
    p.subscribe_slowdown = declare_parameter("subscribe_slowdown", p.subscribe_slowdown);
    p.subscribe_watchdog = declare_parameter("subscribe_watchdog", p.subscribe_watchdog);
    p.subscribe_base_state = declare_parameter("subscribe_base_state", p.subscribe_base_state);
    p.subscribe_motor_board_status = declare_parameter("subscribe_motor_board_status", p.subscribe_motor_board_status);
    p.summary_topic = declare_parameter("summary_topic", p.summary_topic);
    p.heartbeat_topic = declare_parameter("heartbeat_topic", p.heartbeat_topic);
    p.publish_hz = declare_parameter("publish_hz", p.publish_hz);
    p.heartbeat_hz = declare_parameter("heartbeat_hz", p.heartbeat_hz);
    p.cmd_stale_s = declare_parameter("cmd_stale_s", p.cmd_stale_s);
    p.safety_stale_s = declare_parameter("safety_stale_s", p.safety_stale_s);
    p.watchdog_stale_s = declare_parameter("watchdog_stale_s", p.watchdog_stale_s);
    p.base_state_stale_s = declare_parameter("base_state_stale_s", p.base_state_stale_s);
    p.motor_board_status_stale_s = declare_parameter("motor_board_status_stale_s", p.motor_board_status_stale_s);
    p.pretty_json = declare_parameter("pretty_json", p.pretty_json);
    state_ = std::make_unique<summary::StateSummary>(std::move(p), monotonic_now());
    const auto & config = state_->parameters();
    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    const auto sensor = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    summary_pub_ = create_publisher<std_msgs::msg::String>(config.summary_topic, reliable);
    heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>(config.heartbeat_topic, reliable);
    if (config.subscribe_cmd) {
      subscriptions_.push_back(create_subscription<geometry_msgs::msg::Twist>(
        config.cmd_topic, reliable, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          state_->command(msg->linear.x, msg->linear.y, msg->angular.z, monotonic_now());
        }));
    }
    if (config.subscribe_safety_stop) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
        config.safety_stop_topic, sensor, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          state_->safety_stop(msg->data, monotonic_now());
        }));
    }
    if (config.subscribe_slowdown) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::Float32>(
        config.slowdown_topic, sensor, [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
          state_->slowdown(msg->data, monotonic_now());
        }));
    }
    if (config.subscribe_watchdog) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        config.watchdog_topic, reliable, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          state_->watchdog(msg->data, monotonic_now());
        }));
    }
    if (config.subscribe_base_state) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        config.base_state_topic, reliable, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          state_->base_state(msg->data, monotonic_now());
        }));
    }
    if (config.subscribe_motor_board_status) {
      subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        config.motor_board_status_topic, reliable, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          state_->motor_board(msg->data, monotonic_now());
        }));
    }
    // rclpy.create_timer uses the node's ROS clock, not a wall timer. Freshness
    // remains monotonic even when /clock pauses, jumps backwards or speeds up.
    summary_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / config.publish_hz), [this]() {
        state_->serialize(monotonic_now(), summary_message_.data);
        summary_pub_->publish(summary_message_);
        state_->summary_published();
      });
    heartbeat_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / config.heartbeat_hz), [this]() {
        std_msgs::msg::Bool message;
        message.data = true;
        heartbeat_pub_->publish(message);
      });
    RCLCPP_INFO(get_logger(), "base_state_publisher_node started (cpp) | summary_topic=%s publish_hz=%.1f",
      config.summary_topic.c_str(), config.publish_hz);
  }

private:
  std::unique_ptr<summary::StateSummary> state_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr summary_timer_, heartbeat_timer_;
  std_msgs::msg::String summary_message_;
};
}  // namespace savo_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // Keep cache updates and serialization mutually exclusive, as in rclpy.spin.
    rclcpp::spin(std::make_shared<savo_base::BaseStatePublisherNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("base_state_publisher_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
