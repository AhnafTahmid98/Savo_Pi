#include "savo_power/power_health_node.hpp"

#include "savo_power/constants.hpp"
#include "savo_power/topic_names.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace savo_power
{
namespace
{

double clamp_rate_hz(double value)
{
  if (value < 0.1) {
    return 0.1;
  }

  if (value > 10.0) {
    return 10.0;
  }

  return value;
}

}  // namespace

PowerHealthNode::PowerHealthNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node(constants::kPowerHealthNodeName, options)
{
  load_params();

  publisher_ = create_publisher<std_msgs::msg::String>(
    topics::kHealth,
    rclcpp::QoS(10).reliable());

  subscription_ = create_subscription<std_msgs::msg::String>(
    topics::kStatus,
    rclcpp::QoS(10).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      update_status(msg->data);
    });

  const double rate_hz = clamp_rate_hz(params_.publish_rate_hz);
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    [this]() {
      publish_health();
    });

  RCLCPP_INFO(
    get_logger(),
    "Started %s subscribing %s publishing %s",
    constants::kPowerHealthNodeName,
    topics::kStatus,
    topics::kHealth);
}

void PowerHealthNode::load_params()
{
  params_ = PowerHealthNodeParams{};

  params_.publish_rate_hz = declare_parameter<double>(
    params::kPublishRateHz,
    params_.publish_rate_hz);

  params_.stale_timeout_s = declare_parameter<double>(
    params::kStaleTimeoutS,
    params_.stale_timeout_s);

  PowerHealthConfig config;
  config.stale_timeout_s = params_.stale_timeout_s;
  config.missing_status_is_error = true;
  config.stale_status_is_error = true;
  config.unknown_state_is_error = true;

  health_.set_config(config);
}

void PowerHealthNode::update_status(
  const std::string & status_text)
{
  last_status_text_ = status_text;
  last_status_seen_ = now();
  status_seen_ = true;
}

double PowerHealthNode::status_age_s() const
{
  if (!status_seen_) {
    return 0.0;
  }

  return (now() - last_status_seen_).seconds();
}

PowerHealthInput PowerHealthNode::make_health_input() const
{
  PowerHealthInput input;

  input.seen = status_seen_;
  input.age_s = status_age_s();
  input.status_text = last_status_text_;
  input.state = PowerHealth::state_from_status_text(last_status_text_);

  return input;
}

void PowerHealthNode::publish_health()
{
  const auto input = make_health_input();
  const auto result = health_.evaluate(input);

  std_msgs::msg::String message;
  message.data = result.text;

  publisher_->publish(message);

  if (
    result.level == PowerHealthLevel::ERROR ||
    result.level == PowerHealthLevel::UNKNOWN)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Power health error: %s",
      message.data.c_str());
    return;
  }

  if (result.level == PowerHealthLevel::WARN) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Power health warning: %s",
      message.data.c_str());
  }
}

}  // namespace savo_power

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<savo_power::PowerHealthNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
