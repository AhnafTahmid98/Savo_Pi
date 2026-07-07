#include "savo_power/power_aggregator_node.hpp"

#include "savo_power/constants.hpp"
#include "savo_power/topic_names.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
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

std::string lowercase(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

  return value;
}

bool contains_token(
  const std::string & text,
  const std::string & token)
{
  return text.find(token) != std::string::npos;
}

}  // namespace

PowerAggregatorNode::PowerAggregatorNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node(constants::kPowerAggregatorNodeName, options)
{
  core_.source = BatterySource::CORE_UPS;
  edge_.source = BatterySource::EDGE_UPS;
  base_.source = BatterySource::BASE_BATTERY;

  load_params();

  status_publisher_ = create_publisher<std_msgs::msg::String>(
    topics::kStatus,
    rclcpp::QoS(10).reliable());

  dashboard_publisher_ = create_publisher<std_msgs::msg::String>(
    topics::kDashboardText,
    rclcpp::QoS(10).reliable());

  core_subscription_ = create_subscription<std_msgs::msg::String>(
    topics::kCoreUps,
    rclcpp::QoS(10).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      update_source(core_, msg->data);
    });

  edge_subscription_ = create_subscription<std_msgs::msg::String>(
    topics::kEdgeUps,
    rclcpp::QoS(10).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      update_source(edge_, msg->data);
    });

  base_subscription_ = create_subscription<std_msgs::msg::String>(
    topics::kBaseBattery,
    rclcpp::QoS(10).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      update_source(base_, msg->data);
    });

  const double rate_hz = clamp_rate_hz(params_.publish_rate_hz);
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    [this]() {
      publish_status();
    });

  RCLCPP_INFO(
    get_logger(),
    "Started %s subscribing [%s, %s, %s] publishing [%s, %s]",
    constants::kPowerAggregatorNodeName,
    topics::kCoreUps,
    topics::kEdgeUps,
    topics::kBaseBattery,
    topics::kStatus,
    topics::kDashboardText);
}

void PowerAggregatorNode::load_params()
{
  params_ = PowerAggregatorNodeParams{};

  params_.publish_rate_hz = declare_parameter<double>(
    "publish_rate_hz",
    params_.publish_rate_hz);

  params_.stale_timeout_s = declare_parameter<double>(
    "stale_timeout_s",
    params_.stale_timeout_s);

  params_.core_ups_expected = declare_parameter<bool>(
    "core_ups_expected",
    params_.core_ups_expected);

  params_.edge_ups_expected = declare_parameter<bool>(
    "edge_ups_expected",
    params_.edge_ups_expected);

  params_.base_battery_expected = declare_parameter<bool>(
    "base_battery_expected",
    params_.base_battery_expected);

  PowerAggregatorConfig config;
  config.core_ups_expected = params_.core_ups_expected;
  config.edge_ups_expected = params_.edge_ups_expected;
  config.base_battery_expected = params_.base_battery_expected;
  config.stale_timeout_s = params_.stale_timeout_s;

  aggregator_.set_config(config);
}

void PowerAggregatorNode::update_source(
  CachedSource & source,
  const std::string & text)
{
  source.seen = true;
  source.last_seen = now();
  source.text = text;
  source.state = state_from_text(text);
}

PowerAggregatorInputs PowerAggregatorNode::make_inputs() const
{
  PowerAggregatorInputs inputs;

  inputs.core_ups = make_source_input(core_);
  inputs.edge_ups = make_source_input(edge_);
  inputs.base_battery = make_source_input(base_);

  return inputs;
}

TimedPowerSourceInput PowerAggregatorNode::make_source_input(
  const CachedSource & source) const
{
  TimedPowerSourceInput input;

  input.source = source.source;
  input.seen = source.seen;
  input.state = source.state;
  input.age_s = source_age_s(source);
  input.text = source.text;

  return input;
}

double PowerAggregatorNode::source_age_s(
  const CachedSource & source) const
{
  if (!source.seen) {
    return 0.0;
  }

  return (now() - source.last_seen).seconds();
}

void PowerAggregatorNode::publish_status()
{
  const auto inputs = make_inputs();
  const auto summary = aggregator_.aggregate(inputs);

  std_msgs::msg::String status_message;
  status_message.data = summary.status_text;

  std_msgs::msg::String dashboard_message;
  dashboard_message.data = summary.dashboard_text;

  status_publisher_->publish(status_message);
  dashboard_publisher_->publish(dashboard_message);

  if (summary.health_level == PowerHealthLevel::ERROR) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Power aggregate error: %s",
      status_message.data.c_str());
    return;
  }

  if (summary.health_level == PowerHealthLevel::WARN) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Power aggregate warning: %s",
      status_message.data.c_str());
  }
}

PowerState PowerAggregatorNode::state_from_text(
  const std::string & text)
{
  const std::string lower = lowercase(text);

  if (
    contains_token(lower, "overall=critical") ||
    contains_token(lower, "state=critical") ||
    contains_token(lower, " critical"))
  {
    return PowerState::CRITICAL;
  }

  if (
    contains_token(lower, "overall=error") ||
    contains_token(lower, "state=error") ||
    contains_token(lower, " error") ||
    contains_token(lower, "error:"))
  {
    return PowerState::ERROR;
  }

  if (
    contains_token(lower, "overall=stale") ||
    contains_token(lower, "state=stale") ||
    contains_token(lower, " stale"))
  {
    return PowerState::STALE;
  }

  if (
    contains_token(lower, "overall=low") ||
    contains_token(lower, "state=low") ||
    contains_token(lower, " low"))
  {
    return PowerState::LOW;
  }

  if (
    contains_token(lower, "overall=unknown") ||
    contains_token(lower, "state=unknown") ||
    contains_token(lower, " unknown"))
  {
    return PowerState::UNKNOWN;
  }

  if (
    contains_token(lower, "overall=charging") ||
    contains_token(lower, "state=charging") ||
    contains_token(lower, " charging"))
  {
    return PowerState::CHARGING;
  }

  if (
    contains_token(lower, "overall=full") ||
    contains_token(lower, "state=full") ||
    contains_token(lower, " full"))
  {
    return PowerState::FULL;
  }

  if (
    contains_token(lower, "overall=ok") ||
    contains_token(lower, "state=ok") ||
    contains_token(lower, " ok"))
  {
    return PowerState::OK;
  }

  return PowerState::UNKNOWN;
}

}  // namespace savo_power

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<savo_power::PowerAggregatorNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
