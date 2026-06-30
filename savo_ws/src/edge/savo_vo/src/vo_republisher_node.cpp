#include "savo_vo/vo_republisher_node.hpp"

#include <functional>

#include "savo_vo/vo_constants.hpp"

namespace savo_vo
{

VORepublisherNode::VORepublisherNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("vo_republisher_node", options)
{
  declare_parameters();
  load_parameters();
  create_publishers();
  create_subscribers();

  RCLCPP_INFO(
    get_logger(),
    "VO republisher started: %s -> %s",
    input_topic_.c_str(),
    output_topic_.c_str());
}

rclcpp::QoS VORepublisherNode::odometry_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

void VORepublisherNode::declare_parameters()
{
  declare_parameter<std::string>(
    constants::kOdomRawTopicParam,
    constants::kVoOdomRawTopic);

  declare_parameter<std::string>(
    constants::kOdomTopicParam,
    constants::kVoOdomTopic);

  declare_parameter<std::string>(
    constants::kOdomFrameParam,
    constants::kOdomFrame);

  declare_parameter<std::string>(
    constants::kBaseFrameParam,
    constants::kBaseFrame);
}

void VORepublisherNode::load_parameters()
{
  input_topic_ = get_parameter(constants::kOdomRawTopicParam).as_string();
  output_topic_ = get_parameter(constants::kOdomTopicParam).as_string();
  odom_frame_ = get_parameter(constants::kOdomFrameParam).as_string();
  base_frame_ = get_parameter(constants::kBaseFrameParam).as_string();

  if (input_topic_.empty()) {
    input_topic_ = constants::kVoOdomRawTopic;
  }

  if (output_topic_.empty()) {
    output_topic_ = constants::kVoOdomTopic;
  }

  if (odom_frame_.empty()) {
    odom_frame_ = constants::kOdomFrame;
  }

  if (base_frame_.empty()) {
    base_frame_ = constants::kBaseFrame;
  }
}

void VORepublisherNode::create_publishers()
{
  odom_pub_ = create_publisher<Odometry>(
    output_topic_,
    odometry_qos());
}

void VORepublisherNode::create_subscribers()
{
  odom_sub_ = create_subscription<Odometry>(
    input_topic_,
    odometry_qos(),
    std::bind(&VORepublisherNode::on_odom, this, std::placeholders::_1));
}

void VORepublisherNode::on_odom(const Odometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  Odometry output = *msg;
  output.header.frame_id = odom_frame_;
  output.child_frame_id = base_frame_;

  odom_pub_->publish(output);
}

}  // namespace savo_vo
