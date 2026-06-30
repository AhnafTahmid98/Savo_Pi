#include "savo_vo/vo_diagnostics_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>

#include "diagnostic_msgs/msg/key_value.hpp"

#include "savo_vo/vo_constants.hpp"

namespace savo_vo
{
namespace
{

diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

VODiagnosticsNode::VODiagnosticsNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("vo_diagnostics_node", options)
{
  declare_parameters();
  load_parameters();
  create_publishers();
  create_subscribers();
  create_timers();

  RCLCPP_INFO(
    get_logger(),
    "VO diagnostics started: status=%s, health=%s, diagnostics=%s",
    status_topic_.c_str(),
    health_topic_.c_str(),
    diagnostics_topic_.c_str());
}

rclcpp::QoS VODiagnosticsNode::diagnostics_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS VODiagnosticsNode::status_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

void VODiagnosticsNode::declare_parameters()
{
  declare_parameter<std::string>(
    constants::kStatusTopicParam,
    constants::kVoStatusTopic);

  declare_parameter<std::string>(
    constants::kHealthTopicParam,
    constants::kVoHealthTopic);

  declare_parameter<std::string>(
    constants::kDiagnosticsTopicParam,
    constants::kDiagnosticsTopic);

  declare_parameter<double>(
    constants::kDiagnosticsRateHzParam,
    constants::kDefaultDiagnosticsRateHz);
}

void VODiagnosticsNode::load_parameters()
{
  status_topic_ = get_parameter(constants::kStatusTopicParam).as_string();
  health_topic_ = get_parameter(constants::kHealthTopicParam).as_string();
  diagnostics_topic_ = get_parameter(constants::kDiagnosticsTopicParam).as_string();
  diagnostics_rate_hz_ = get_parameter(constants::kDiagnosticsRateHzParam).as_double();

  if (status_topic_.empty()) {
    status_topic_ = constants::kVoStatusTopic;
  }

  if (health_topic_.empty()) {
    health_topic_ = constants::kVoHealthTopic;
  }

  if (diagnostics_topic_.empty()) {
    diagnostics_topic_ = constants::kDiagnosticsTopic;
  }

  diagnostics_rate_hz_ = std::max(0.20, diagnostics_rate_hz_);
}

void VODiagnosticsNode::create_publishers()
{
  diagnostics_pub_ = create_publisher<DiagnosticArray>(
    diagnostics_topic_,
    diagnostics_qos());
}

void VODiagnosticsNode::create_subscribers()
{
  status_sub_ = create_subscription<String>(
    status_topic_,
    status_qos(),
    std::bind(&VODiagnosticsNode::on_status, this, std::placeholders::_1));

  health_sub_ = create_subscription<String>(
    health_topic_,
    status_qos(),
    std::bind(&VODiagnosticsNode::on_health, this, std::placeholders::_1));
}

void VODiagnosticsNode::create_timers()
{
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / diagnostics_rate_hz_));

  timer_ = create_wall_timer(
    period,
    std::bind(&VODiagnosticsNode::publish_diagnostics, this));
}

void VODiagnosticsNode::on_status(const String::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  last_status_ = msg->data.empty() ? "status empty" : msg->data;
}

void VODiagnosticsNode::on_health(const String::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  last_health_ = msg->data.empty() ? "health empty" : msg->data;
}

void VODiagnosticsNode::publish_diagnostics()
{
  DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(build_diagnostic_status());

  diagnostics_pub_->publish(array);
}

VODiagnosticsNode::DiagnosticStatus VODiagnosticsNode::build_diagnostic_status() const
{
  DiagnosticStatus status;
  status.name = "savo_vo";
  status.hardware_id = "robot_savo_visual_odometry";

  const std::string combined = last_health_ + " " + last_status_;

  if (combined.find("error") != std::string::npos) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "visual odometry error";
  } else if (
    combined.find("stale") != std::string::npos ||
    combined.find("lost") != std::string::npos ||
    combined.find("rejected") != std::string::npos ||
    combined.find("degraded") != std::string::npos)
  {
    status.level = DiagnosticStatus::WARN;
    status.message = "visual odometry degraded";
  } else if (combined.find("ok") != std::string::npos) {
    status.level = DiagnosticStatus::OK;
    status.message = "visual odometry healthy";
  } else {
    status.level = DiagnosticStatus::WARN;
    status.message = "visual odometry waiting";
  }

  status.values.push_back(make_key_value("vo_status", last_status_));
  status.values.push_back(make_key_value("vo_health", last_health_));

  return status;
}

}  // namespace savo_vo
