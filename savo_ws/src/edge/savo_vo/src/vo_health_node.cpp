#include "savo_vo/vo_health_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

#include "savo_vo/vo_constants.hpp"

namespace savo_vo
{

VOHealthNode::VOHealthNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("vo_health_node", options)
{
  declare_parameters();
  load_parameters();
  create_publishers();
  create_subscribers();
  create_timers();

  RCLCPP_INFO(
    get_logger(),
    "VO health monitor started: odom=%s, status=%s, health=%s",
    odom_topic_.c_str(),
    status_topic_.c_str(),
    health_topic_.c_str());
}

rclcpp::QoS VOHealthNode::odometry_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS VOHealthNode::status_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

void VOHealthNode::declare_parameters()
{
  declare_parameter<std::string>(
    constants::kOdomTopicParam,
    constants::kVoOdomTopic);

  declare_parameter<std::string>(
    constants::kStatusTopicParam,
    constants::kVoStatusTopic);

  declare_parameter<std::string>(
    constants::kHealthTopicParam,
    constants::kVoHealthTopic);

  declare_parameter<double>(
    constants::kStaleTimeoutSParam,
    constants::kDefaultStaleTimeoutS);
}

void VOHealthNode::load_parameters()
{
  odom_topic_ = get_parameter(constants::kOdomTopicParam).as_string();
  status_topic_ = get_parameter(constants::kStatusTopicParam).as_string();
  health_topic_ = get_parameter(constants::kHealthTopicParam).as_string();
  stale_timeout_s_ = get_parameter(constants::kStaleTimeoutSParam).as_double();

  if (odom_topic_.empty()) {
    odom_topic_ = constants::kVoOdomTopic;
  }

  if (status_topic_.empty()) {
    status_topic_ = constants::kVoStatusTopic;
  }

  if (health_topic_.empty()) {
    health_topic_ = constants::kVoHealthTopic;
  }

  stale_timeout_s_ = std::max(0.10, stale_timeout_s_);
}

void VOHealthNode::create_publishers()
{
  health_pub_ = create_publisher<String>(
    health_topic_,
    status_qos());
}

void VOHealthNode::create_subscribers()
{
  odom_sub_ = create_subscription<Odometry>(
    odom_topic_,
    odometry_qos(),
    std::bind(&VOHealthNode::on_odom, this, std::placeholders::_1));

  status_sub_ = create_subscription<String>(
    status_topic_,
    status_qos(),
    std::bind(&VOHealthNode::on_status, this, std::placeholders::_1));
}

void VOHealthNode::create_timers()
{
  timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&VOHealthNode::publish_health, this));
}

void VOHealthNode::on_odom(const Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  has_odom_ = true;
  last_odom_time_s_ = now().seconds();
}

void VOHealthNode::on_status(const String::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  last_status_ = msg->data.empty() ? "status empty" : msg->data;
}

void VOHealthNode::publish_health()
{
  String message;
  message.data = build_health_message(now().seconds());
  health_pub_->publish(message);
}

std::string VOHealthNode::build_health_message(const double now_s) const
{
  if (!has_odom_) {
    return "waiting: no visual odometry received";
  }

  const double odom_age_s = std::max(0.0, now_s - last_odom_time_s_);

  if (odom_age_s > stale_timeout_s_) {
    return "stale: visual odometry timeout";
  }

  if (last_status_.find("error") != std::string::npos) {
    return "error: " + last_status_;
  }

  if (last_status_.find("lost") != std::string::npos ||
      last_status_.find("rejected") != std::string::npos) {
    return "degraded: " + last_status_;
  }

  return "ok: " + last_status_;
}

}  // namespace savo_vo
