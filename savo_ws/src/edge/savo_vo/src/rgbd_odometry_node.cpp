#include "savo_vo/rgbd_odometry_node.hpp"

#include <algorithm>
#include <functional>
#include <string>

#include "savo_vo/vo_constants.hpp"

namespace savo_vo
{

RGBDOdometryNode::RGBDOdometryNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("rgbd_odometry_node", options)
{
  declare_parameters();
  load_parameters();
  create_publishers();
  create_subscribers();

  publish_status("waiting for RGB-D input");
  publish_health("waiting");

  RCLCPP_INFO(
    get_logger(),
    "RGB-D odometry node started: color=%s, depth=%s, odom=%s",
    color_image_topic_.c_str(),
    depth_image_topic_.c_str(),
    odom_topic_.c_str());
}

rclcpp::QoS RGBDOdometryNode::camera_qos()
{
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  return qos;
}

rclcpp::QoS RGBDOdometryNode::odometry_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS RGBDOdometryNode::status_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

void RGBDOdometryNode::declare_parameters()
{
  declare_parameter<std::string>(
    constants::kColorImageTopicParam,
    constants::kColorImageTopic);

  declare_parameter<std::string>(
    constants::kColorCameraInfoTopicParam,
    constants::kColorCameraInfoTopic);

  declare_parameter<std::string>(
    constants::kDepthImageTopicParam,
    constants::kDepthImageTopic);

  declare_parameter<std::string>(
    constants::kDepthCameraInfoTopicParam,
    constants::kDepthCameraInfoTopic);

  declare_parameter<std::string>(
    constants::kOdomTopicParam,
    constants::kVoOdomRawTopic);

  declare_parameter<std::string>(
    constants::kStatusTopicParam,
    constants::kVoStatusTopic);

  declare_parameter<std::string>(
    constants::kHealthTopicParam,
    constants::kVoHealthTopic);

  declare_parameter<std::string>(
    constants::kTrackingQualityTopicParam,
    constants::kVoTrackingQualityTopic);

  declare_parameter<std::string>(
    constants::kOdomFrameParam,
    constants::kOdomFrame);

  declare_parameter<std::string>(
    constants::kBaseFrameParam,
    constants::kBaseFrame);

  declare_parameter<std::string>(
    constants::kCameraFrameParam,
    constants::kCameraFrame);

  declare_parameter<double>("max_image_delay_s", 0.15);
  declare_parameter<double>("max_depth_delay_s", 0.15);

  declare_parameter<int>("min_features", 80);
  declare_parameter<int>("good_features_target", 300);
  declare_parameter<int>("max_features", 800);

  declare_parameter<double>(
    constants::kMinTrackingQualityParam,
    constants::kDefaultMinTrackingQuality);

  declare_parameter<double>("max_translation_jump_m", 0.30);
  declare_parameter<double>("max_rotation_jump_rad", 0.35);

  declare_parameter<bool>(
    constants::kPublishTfParam,
    constants::kDefaultPublishTf);

  declare_parameter<bool>("publish_diagnostics", true);
}

void RGBDOdometryNode::load_parameters()
{
  color_image_topic_ = get_parameter(constants::kColorImageTopicParam).as_string();
  color_camera_info_topic_ = get_parameter(constants::kColorCameraInfoTopicParam).as_string();
  depth_image_topic_ = get_parameter(constants::kDepthImageTopicParam).as_string();
  depth_camera_info_topic_ = get_parameter(constants::kDepthCameraInfoTopicParam).as_string();

  odom_topic_ = get_parameter(constants::kOdomTopicParam).as_string();
  status_topic_ = get_parameter(constants::kStatusTopicParam).as_string();
  health_topic_ = get_parameter(constants::kHealthTopicParam).as_string();
  tracking_quality_topic_ = get_parameter(constants::kTrackingQualityTopicParam).as_string();

  odom_frame_ = get_parameter(constants::kOdomFrameParam).as_string();
  base_frame_ = get_parameter(constants::kBaseFrameParam).as_string();
  camera_frame_ = get_parameter(constants::kCameraFrameParam).as_string();

  max_image_delay_s_ = std::max(0.01, get_parameter("max_image_delay_s").as_double());
  max_depth_delay_s_ = std::max(0.01, get_parameter("max_depth_delay_s").as_double());

  min_features_ = std::max(1, static_cast<int>(get_parameter("min_features").as_int()));
  good_features_target_ = std::max(1, static_cast<int>(get_parameter("good_features_target").as_int()));
  max_features_ = std::max(min_features_, static_cast<int>(get_parameter("max_features").as_int()));

  min_tracking_quality_ = std::clamp(
    get_parameter(constants::kMinTrackingQualityParam).as_double(),
    0.0,
    1.0);

  max_translation_jump_m_ = std::max(0.01, get_parameter("max_translation_jump_m").as_double());
  max_rotation_jump_rad_ = std::max(0.01, get_parameter("max_rotation_jump_rad").as_double());

  publish_tf_ = get_parameter(constants::kPublishTfParam).as_bool();
  publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();

  if (odom_topic_.empty()) {
    odom_topic_ = constants::kVoOdomRawTopic;
  }

  if (status_topic_.empty()) {
    status_topic_ = constants::kVoStatusTopic;
  }

  if (health_topic_.empty()) {
    health_topic_ = constants::kVoHealthTopic;
  }

  if (tracking_quality_topic_.empty()) {
    tracking_quality_topic_ = constants::kVoTrackingQualityTopic;
  }
}

void RGBDOdometryNode::create_publishers()
{
  odom_pub_ = create_publisher<Odometry>(
    odom_topic_,
    odometry_qos());

  status_pub_ = create_publisher<String>(
    status_topic_,
    status_qos());

  health_pub_ = create_publisher<String>(
    health_topic_,
    status_qos());

  tracking_quality_pub_ = create_publisher<Float32>(
    tracking_quality_topic_,
    status_qos());
}

void RGBDOdometryNode::create_subscribers()
{
  color_sub_ = create_subscription<Image>(
    color_image_topic_,
    camera_qos(),
    std::bind(&RGBDOdometryNode::on_color_image, this, std::placeholders::_1));

  depth_sub_ = create_subscription<Image>(
    depth_image_topic_,
    camera_qos(),
    std::bind(&RGBDOdometryNode::on_depth_image, this, std::placeholders::_1));

  camera_info_sub_ = create_subscription<CameraInfo>(
    color_camera_info_topic_,
    camera_qos(),
    std::bind(&RGBDOdometryNode::on_camera_info, this, std::placeholders::_1));
}

void RGBDOdometryNode::on_color_image(const Image::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  latest_color_ = msg;
  try_process_frame();
}

void RGBDOdometryNode::on_depth_image(const Image::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  latest_depth_ = msg;
  try_process_frame();
}

void RGBDOdometryNode::on_camera_info(const CameraInfo::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  latest_camera_info_ = msg;
  try_process_frame();
}

void RGBDOdometryNode::try_process_frame()
{
  if (!has_required_inputs()) {
    publish_status("waiting for synchronized RGB-D inputs");
    publish_health("waiting");
    publish_tracking_quality(0.0);
    return;
  }

  const auto intrinsics = camera_intrinsics_from_info(*latest_camera_info_);

  if (!intrinsics.is_valid()) {
    publish_status("error: invalid camera intrinsics");
    publish_health("error");
    publish_tracking_quality(0.0);
    return;
  }

  publish_status("tracking: RGB-D input ready");
  publish_health("ok");
  publish_tracking_quality(1.0);
}

bool RGBDOdometryNode::has_required_inputs() const
{
  return latest_color_ != nullptr &&
         latest_depth_ != nullptr &&
         latest_camera_info_ != nullptr;
}

CameraIntrinsics RGBDOdometryNode::camera_intrinsics_from_info(
  const CameraInfo & info) const
{
  CameraIntrinsics intrinsics;
  intrinsics.fx = info.k[0];
  intrinsics.fy = info.k[4];
  intrinsics.cx = info.k[2];
  intrinsics.cy = info.k[5];
  intrinsics.width = static_cast<int>(info.width);
  intrinsics.height = static_cast<int>(info.height);

  return intrinsics;
}

void RGBDOdometryNode::publish_odometry(const Odometry & odometry)
{
  odom_pub_->publish(odometry);
}

void RGBDOdometryNode::publish_status(const std::string & message)
{
  String status;
  status.data = message;
  status_pub_->publish(status);
}

void RGBDOdometryNode::publish_health(const std::string & message)
{
  String health;
  health.data = message;
  health_pub_->publish(health);
}

void RGBDOdometryNode::publish_tracking_quality(const double score)
{
  Float32 quality;
  quality.data = static_cast<float>(std::clamp(score, 0.0, 1.0));
  tracking_quality_pub_->publish(quality);
}

}  // namespace savo_vo
