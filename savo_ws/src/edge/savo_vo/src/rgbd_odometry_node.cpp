#include "savo_vo/rgbd_odometry_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "rmw/qos_profiles.h"

#include "savo_vo/geometry_utils.hpp"
#include "savo_vo/timestamp_sync.hpp"
#include "savo_vo/vo_constants.hpp"

namespace savo_vo
{
namespace
{

constexpr double kHalfPi = 1.5707963267948966;

std::string rejection_message(
  const TrackingQuality & quality,
  const std::string & reason)
{
  return build_tracking_message(
    VOTrackingState::kRejected,
    quality.feature_count,
    quality.tracked_count,
    quality.valid_depth_count,
    quality.pnp_inlier_count,
    quality.pnp_inlier_ratio,
    quality.score) + ", accepted=false, reason=" + reason;
}

}  // namespace

RGBDOdometryNode::RGBDOdometryNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("rgbd_odometry_node", options)
{
  declare_parameters();
  load_parameters();
  create_publishers();
  create_subscribers();
  create_timers();

  publish_status("waiting for synchronized aligned RGB-D input");
  publish_tracking_quality(0.0);

  RCLCPP_INFO(
    get_logger(),
    "C++ RGB-D PnP odometry started: color=%s, aligned_depth=%s, odom=%s, base=%s",
    color_image_topic_.c_str(),
    depth_image_topic_.c_str(),
    odom_topic_.c_str(),
    base_frame_.c_str());
}

rclcpp::QoS RGBDOdometryNode::camera_qos()
{
  return rclcpp::SensorDataQoS();
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

  declare_parameter<std::string>(constants::kOdomTopicParam, constants::kVoOdomRawTopic);
  declare_parameter<std::string>(constants::kStatusTopicParam, constants::kVoStatusTopic);
  declare_parameter<std::string>(
    constants::kTrackingQualityTopicParam,
    constants::kVoTrackingQualityTopic);

  declare_parameter<std::string>(constants::kOdomFrameParam, constants::kOdomFrame);
  declare_parameter<std::string>(constants::kBaseFrameParam, constants::kBaseFrame);
  declare_parameter<std::string>(constants::kCameraFrameParam, constants::kCameraFrame);

  declare_parameter<int>("sync_queue_size", 10);
  declare_parameter<double>("max_sync_delta_s", 0.02);
  declare_parameter<double>("max_frame_interval_s", 0.20);

  declare_parameter<int>("min_features", 80);
  declare_parameter<int>("good_features_target", 300);
  declare_parameter<int>("max_features", 800);
  declare_parameter<double>(
    constants::kMinTrackingQualityParam,
    constants::kDefaultMinTrackingQuality);

  declare_parameter<int>("min_depth_correspondences", 30);
  declare_parameter<int>("min_pnp_inliers", 20);
  declare_parameter<double>("min_pnp_inlier_ratio", 0.50);
  declare_parameter<int>("pnp_iterations", 100);
  declare_parameter<double>("pnp_reprojection_error_px", 2.5);
  declare_parameter<double>("pnp_confidence", 0.99);

  declare_parameter<double>("depth_scale_16u", 0.001);
  declare_parameter<double>("min_depth_m", 0.20);
  declare_parameter<double>("max_depth_m", 6.0);

  declare_parameter<double>("max_translation_jump_m", 0.15);
  declare_parameter<double>("max_rotation_jump_rad", 0.20);
  declare_parameter<double>("translation_deadband_m", 0.001);
  declare_parameter<double>("rotation_deadband_rad", 0.001);

  // These values are the current URDF transform from base_footprint to
  // camera_color_optical_frame: base_link is 0.0325 m above the footprint,
  // and the camera mount is (0.130, 0, 0.1925) m relative to base_link.
  declare_parameter<double>("base_to_camera_x_m", 0.130);
  declare_parameter<double>("base_to_camera_y_m", 0.0);
  declare_parameter<double>("base_to_camera_z_m", 0.225);
  declare_parameter<double>("base_to_camera_roll_rad", -kHalfPi);
  declare_parameter<double>("base_to_camera_pitch_rad", 0.0);
  declare_parameter<double>("base_to_camera_yaw_rad", -kHalfPi);

  declare_parameter<double>("position_variance", 0.05);
  declare_parameter<double>("yaw_variance", 0.10);
  declare_parameter<double>("linear_velocity_variance", 0.10);
  declare_parameter<double>("angular_velocity_variance", 0.20);

  declare_parameter<bool>(constants::kPublishTfParam, constants::kDefaultPublishTf);
  declare_parameter<bool>("publish_diagnostics", true);
}

void RGBDOdometryNode::load_parameters()
{
  color_image_topic_ = get_parameter(constants::kColorImageTopicParam).as_string();
  color_camera_info_topic_ = get_parameter(constants::kColorCameraInfoTopicParam).as_string();
  depth_image_topic_ = get_parameter(constants::kDepthImageTopicParam).as_string();

  odom_topic_ = get_parameter(constants::kOdomTopicParam).as_string();
  status_topic_ = get_parameter(constants::kStatusTopicParam).as_string();
  tracking_quality_topic_ = get_parameter(constants::kTrackingQualityTopicParam).as_string();

  odom_frame_ = get_parameter(constants::kOdomFrameParam).as_string();
  base_frame_ = get_parameter(constants::kBaseFrameParam).as_string();
  camera_frame_ = get_parameter(constants::kCameraFrameParam).as_string();

  sync_queue_size_ = std::max(2, static_cast<int>(get_parameter("sync_queue_size").as_int()));
  max_sync_delta_s_ = std::max(0.001, get_parameter("max_sync_delta_s").as_double());
  max_frame_interval_s_ = std::max(
    0.01,
    get_parameter("max_frame_interval_s").as_double());

  min_features_ = std::max(4, static_cast<int>(get_parameter("min_features").as_int()));
  good_features_target_ = std::max(
    min_features_,
    static_cast<int>(get_parameter("good_features_target").as_int()));
  max_features_ = std::max(
    good_features_target_,
    static_cast<int>(get_parameter("max_features").as_int()));
  min_tracking_quality_ = std::clamp(
    get_parameter(constants::kMinTrackingQualityParam).as_double(),
    0.0,
    1.0);

  min_depth_correspondences_ = std::max(
    4,
    static_cast<int>(get_parameter("min_depth_correspondences").as_int()));
  min_pnp_inliers_ = std::max(
    4,
    static_cast<int>(get_parameter("min_pnp_inliers").as_int()));
  min_pnp_inlier_ratio_ = std::clamp(
    get_parameter("min_pnp_inlier_ratio").as_double(),
    0.0,
    1.0);
  pnp_iterations_ = std::max(1, static_cast<int>(get_parameter("pnp_iterations").as_int()));
  pnp_reprojection_error_px_ = std::max(
    0.1,
    get_parameter("pnp_reprojection_error_px").as_double());
  pnp_confidence_ = std::clamp(get_parameter("pnp_confidence").as_double(), 0.01, 0.9999);

  depth_scale_16u_ = std::max(1e-9, get_parameter("depth_scale_16u").as_double());
  min_depth_m_ = std::max(0.01, get_parameter("min_depth_m").as_double());
  max_depth_m_ = std::max(min_depth_m_, get_parameter("max_depth_m").as_double());

  max_translation_jump_m_ = std::max(
    0.001,
    get_parameter("max_translation_jump_m").as_double());
  max_rotation_jump_rad_ = std::max(
    0.001,
    get_parameter("max_rotation_jump_rad").as_double());
  translation_deadband_m_ = std::max(
    0.0,
    get_parameter("translation_deadband_m").as_double());
  rotation_deadband_rad_ = std::max(
    0.0,
    get_parameter("rotation_deadband_rad").as_double());

  base_T_camera_ = make_rigid_transform_from_rpy(
    get_parameter("base_to_camera_x_m").as_double(),
    get_parameter("base_to_camera_y_m").as_double(),
    get_parameter("base_to_camera_z_m").as_double(),
    get_parameter("base_to_camera_roll_rad").as_double(),
    get_parameter("base_to_camera_pitch_rad").as_double(),
    get_parameter("base_to_camera_yaw_rad").as_double());

  covariance_config_.position_variance = std::max(
    1e-6,
    get_parameter("position_variance").as_double());
  covariance_config_.yaw_variance = std::max(
    1e-6,
    get_parameter("yaw_variance").as_double());
  covariance_config_.linear_velocity_variance = std::max(
    1e-6,
    get_parameter("linear_velocity_variance").as_double());
  covariance_config_.angular_velocity_variance = std::max(
    1e-6,
    get_parameter("angular_velocity_variance").as_double());

  publish_tf_ = get_parameter(constants::kPublishTfParam).as_bool();
  publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();

  if (publish_tf_) {
    RCLCPP_WARN(
      get_logger(),
      "publish_tf requested but savo_vo never publishes odom TF; the localization EKF owns it");
    publish_tf_ = false;
  }
}

void RGBDOdometryNode::create_publishers()
{
  odom_pub_ = create_publisher<Odometry>(odom_topic_, odometry_qos());
  status_pub_ = create_publisher<String>(status_topic_, status_qos());
  tracking_quality_pub_ = create_publisher<Float32>(tracking_quality_topic_, status_qos());
}

void RGBDOdometryNode::create_subscribers()
{
  color_sub_.subscribe(this, color_image_topic_, rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, depth_image_topic_, rmw_qos_profile_sensor_data);

  image_sync_ = std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(
    ImageSyncPolicy(sync_queue_size_),
    color_sub_,
    depth_sub_);
  image_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_delta_s_));
  image_sync_->registerCallback(
    std::bind(
      &RGBDOdometryNode::on_synchronized_images,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  camera_info_sub_ = create_subscription<CameraInfo>(
    color_camera_info_topic_,
    camera_qos(),
    std::bind(&RGBDOdometryNode::on_camera_info, this, std::placeholders::_1));
}

void RGBDOdometryNode::create_timers()
{
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RGBDOdometryNode::publish_waiting_status, this));
}

void RGBDOdometryNode::on_synchronized_images(
  const Image::ConstSharedPtr & color_msg,
  const Image::ConstSharedPtr & aligned_depth_msg)
{
  if (!color_msg || !aligned_depth_msg || !latest_camera_info_) {
    publish_status("waiting for synchronized aligned RGB-D input and color CameraInfo");
    publish_tracking_quality(0.0);
    return;
  }

  const double pair_delta_s = stamp_delta_seconds(
    color_msg->header.stamp,
    aligned_depth_msg->header.stamp);
  if (!std::isfinite(pair_delta_s) || pair_delta_s > max_sync_delta_s_) {
    publish_status("rejected: synchronized RGB-D timestamp delta exceeded limit");
    publish_tracking_quality(0.0);
    return;
  }

  if ((!color_msg->header.frame_id.empty() && color_msg->header.frame_id != camera_frame_) ||
    (!aligned_depth_msg->header.frame_id.empty() &&
    aligned_depth_msg->header.frame_id != camera_frame_))
  {
    publish_status("rejected: RGB-D frames do not match configured color optical frame");
    publish_tracking_quality(0.0);
    return;
  }

  const RGBDCameraModel camera_model = camera_model_from_info(*latest_camera_info_);
  if (!camera_model.is_valid()) {
    publish_status("error: unsupported or invalid color CameraInfo calibration");
    publish_tracking_quality(0.0);
    return;
  }

  if (color_msg->width != aligned_depth_msg->width ||
    color_msg->height != aligned_depth_msg->height ||
    static_cast<int>(color_msg->width) != camera_model.width ||
    static_cast<int>(color_msg->height) != camera_model.height)
  {
    publish_status("rejected: aligned depth, color image, and CameraInfo dimensions differ");
    publish_tracking_quality(0.0);
    return;
  }

  cv::Mat gray_image;
  cv::Mat aligned_depth_image;
  if (!convert_images(
      color_msg,
      aligned_depth_msg,
      gray_image,
      aligned_depth_image))
  {
    publish_status("error: failed to convert synchronized RGB-D images");
    publish_tracking_quality(0.0);
    return;
  }

  Odometry odometry;
  const auto quality = estimate_visual_motion(
    gray_image,
    aligned_depth_image,
    camera_model,
    rclcpp::Time(color_msg->header.stamp),
    odometry);

  publish_tracking_quality(quality.score);
  publish_status(quality.message);

  if (quality.is_good(min_tracking_quality_, min_features_)) {
    publish_odometry(odometry);
  }
}

void RGBDOdometryNode::on_camera_info(const CameraInfo::SharedPtr msg)
{
  if (msg) {
    latest_camera_info_ = msg;
  }
}

void RGBDOdometryNode::publish_waiting_status()
{
  if (has_previous_frame_) {
    return;
  }

  publish_status(
    latest_camera_info_ ?
    "waiting_for_reference: waiting for synchronized RGB-D pair" :
    "waiting for color CameraInfo and synchronized aligned RGB-D input");
  publish_tracking_quality(0.0);
}

RGBDCameraModel RGBDOdometryNode::camera_model_from_info(const CameraInfo & info) const
{
  RGBDCameraModel model;
  model.camera_matrix = cv::Matx33d(
    info.k[0], info.k[1], info.k[2],
    info.k[3], info.k[4], info.k[5],
    info.k[6], info.k[7], info.k[8]);
  model.distortion_model = info.distortion_model;
  model.width = static_cast<int>(info.width);
  model.height = static_cast<int>(info.height);

  if (!info.d.empty()) {
    model.distortion_coefficients = cv::Mat(
      1,
      static_cast<int>(info.d.size()),
      CV_64F);
    for (std::size_t index = 0; index < info.d.size(); ++index) {
      model.distortion_coefficients.at<double>(0, static_cast<int>(index)) = info.d[index];
    }
  }

  return model;
}

bool RGBDOdometryNode::convert_images(
  const Image::ConstSharedPtr & color_msg,
  const Image::ConstSharedPtr & depth_msg,
  cv::Mat & gray_image,
  cv::Mat & depth_image) const
{
  try {
    const auto color_cv = cv_bridge::toCvShare(color_msg);
    const auto depth_cv = cv_bridge::toCvShare(depth_msg);
    const cv::Mat & color = color_cv->image;

    if (color.empty() || depth_cv->image.empty()) {
      return false;
    }

    if (color.channels() == 1) {
      gray_image = color.clone();
    } else if (color.channels() == 3) {
      const int conversion = color_msg->encoding == "rgb8" ?
        cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY;
      cv::cvtColor(color, gray_image, conversion);
    } else if (color.channels() == 4) {
      const int conversion = color_msg->encoding == "rgba8" ?
        cv::COLOR_RGBA2GRAY : cv::COLOR_BGRA2GRAY;
      cv::cvtColor(color, gray_image, conversion);
    } else {
      return false;
    }

    depth_image = depth_cv->image.clone();
    return !gray_image.empty() && !depth_image.empty();
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "cv_bridge conversion failed: %s",
      error.what());
    return false;
  }
}

TrackingQuality RGBDOdometryNode::estimate_visual_motion(
  const cv::Mat & gray_image,
  const cv::Mat & aligned_depth_image,
  const RGBDCameraModel & camera_model,
  const rclcpp::Time & stamp,
  Odometry & odometry)
{
  const double current_stamp_s = stamp.seconds();

  if (!has_previous_frame_) {
    reset_reference(gray_image, aligned_depth_image, current_stamp_s);

    TrackingQuality quality;
    quality.feature_count = static_cast<int>(previous_features_.size());
    quality.state = VOTrackingState::kWaitingForReference;
    quality.message = "waiting_for_reference: first synchronized RGB-D frame stored";
    return quality;
  }

  const double dt_s = current_stamp_s - previous_stamp_s_;
  if (!std::isfinite(dt_s) || dt_s <= 0.001 || dt_s > max_frame_interval_s_) {
    reset_reference(gray_image, aligned_depth_image, current_stamp_s);

    TrackingQuality quality;
    quality.state = VOTrackingState::kRejected;
    quality.message = "rejected: invalid RGB-D frame interval; reference reseeded";
    return quality;
  }

  if (previous_features_.size() < static_cast<std::size_t>(min_features_)) {
    previous_features_ = detect_reference_features(previous_gray_image_);
  }

  if (previous_features_.empty()) {
    reset_reference(gray_image, aligned_depth_image, current_stamp_s);

    TrackingQuality quality;
    quality.state = VOTrackingState::kRejected;
    quality.message = "rejected: no reference features; reference reseeded";
    return quality;
  }

  std::vector<cv::Point2f> current_features;
  std::vector<unsigned char> tracking_status;
  std::vector<float> tracking_errors;
  cv::calcOpticalFlowPyrLK(
    previous_gray_image_,
    gray_image,
    previous_features_,
    current_features,
    tracking_status,
    tracking_errors);

  std::vector<cv::Point2f> previous_good;
  std::vector<cv::Point2f> current_good;
  previous_good.reserve(previous_features_.size());
  current_good.reserve(current_features.size());

  for (std::size_t index = 0; index < tracking_status.size(); ++index) {
    if (!tracking_status[index]) {
      continue;
    }

    const auto & previous = previous_features_[index];
    const auto & current = current_features[index];
    if (previous.x < 0.0F || previous.y < 0.0F ||
      previous.x >= static_cast<float>(previous_gray_image_.cols) ||
      previous.y >= static_cast<float>(previous_gray_image_.rows) ||
      current.x < 0.0F || current.y < 0.0F ||
      current.x >= static_cast<float>(gray_image.cols) ||
      current.y >= static_cast<float>(gray_image.rows))
    {
      continue;
    }

    previous_good.push_back(previous);
    current_good.push_back(current);
  }

  const RGBDCorrespondences correspondences = build_rgbd_correspondences(
    previous_good,
    current_good,
    previous_aligned_depth_image_,
    camera_model,
    depth_scale_16u_,
    min_depth_m_,
    max_depth_m_);

  PnPConfig pnp_config;
  pnp_config.minimum_correspondences = min_depth_correspondences_;
  pnp_config.minimum_inliers = min_pnp_inliers_;
  pnp_config.minimum_inlier_ratio = min_pnp_inlier_ratio_;
  pnp_config.iterations = pnp_iterations_;
  pnp_config.reprojection_error_px = pnp_reprojection_error_px_;
  pnp_config.confidence = pnp_confidence_;

  const PnPEstimate pnp = estimate_current_camera_from_previous_camera(
    correspondences,
    camera_model,
    pnp_config);

  TrackingQualityEstimator quality_estimator(
    min_features_,
    good_features_target_,
    min_tracking_quality_);
  TrackingQuality quality = quality_estimator.evaluate(
    static_cast<int>(previous_features_.size()),
    static_cast<int>(current_good.size()),
    pnp.correspondence_count,
    pnp.inlier_count);

  if (!pnp.success) {
    quality.state = VOTrackingState::kRejected;
    quality.message = rejection_message(quality, pnp.rejection_reason);
    reset_reference(gray_image, aligned_depth_image, current_stamp_s);
    return quality;
  }

  if (!quality.is_good(min_tracking_quality_, min_features_)) {
    quality.message = rejection_message(quality, "geometric tracking quality below threshold");
    quality.state = VOTrackingState::kRejected;
    reset_reference(gray_image, aligned_depth_image, current_stamp_s);
    return quality;
  }

  const cv::Matx44d previous_base_T_current_base =
    base_increment_from_camera_increment(
    base_T_camera_,
    pnp.previous_camera_T_current_camera);
  PlanarMotion base_increment = planar_motion_from_transform(
    previous_base_T_current_base);

  if (!motion_is_within_limits(
      base_increment,
      previous_base_T_current_base,
      max_translation_jump_m_,
      max_rotation_jump_rad_))
  {
    quality.state = VOTrackingState::kRejected;
    quality.message = rejection_message(quality, "rigid base motion exceeds configured limits");
    reset_reference(gray_image, aligned_depth_image, current_stamp_s);
    return quality;
  }

  base_increment = suppress_vibration_motion(
    base_increment,
    translation_deadband_m_,
    rotation_deadband_rad_);

  pose_ = compose_pose(pose_, base_increment);

  const double vx = base_increment.x_m / dt_s;
  const double vy = base_increment.y_m / dt_s;
  const double wz = base_increment.yaw_rad / dt_s;
  odometry = build_odometry_message(vx, vy, wz, quality.score, stamp);

  quality.state = VOTrackingState::kTracking;
  quality.message = build_tracking_message(
    quality.state,
    quality.feature_count,
    quality.tracked_count,
    quality.valid_depth_count,
    quality.pnp_inlier_count,
    quality.pnp_inlier_ratio,
    quality.score) + ", accepted=true";

  reset_reference(gray_image, aligned_depth_image, current_stamp_s);
  return quality;
}

void RGBDOdometryNode::reset_reference(
  const cv::Mat & gray_image,
  const cv::Mat & aligned_depth_image,
  const double stamp_s)
{
  previous_gray_image_ = gray_image.clone();
  previous_aligned_depth_image_ = aligned_depth_image.clone();
  previous_features_ = detect_reference_features(previous_gray_image_);
  previous_stamp_s_ = stamp_s;
  has_previous_frame_ = true;
}

std::vector<cv::Point2f> RGBDOdometryNode::detect_reference_features(
  const cv::Mat & gray_image) const
{
  std::vector<cv::Point2f> features;
  if (!gray_image.empty()) {
    cv::goodFeaturesToTrack(gray_image, features, max_features_, 0.01, 8.0);
  }
  return features;
}

RGBDOdometryNode::Odometry RGBDOdometryNode::build_odometry_message(
  const double vx,
  const double vy,
  const double wz,
  const double quality_score,
  const rclcpp::Time & stamp)
{
  Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_;
  odometry.child_frame_id = base_frame_;

  odometry.pose.pose.position.x = pose_.x_m;
  odometry.pose.pose.position.y = pose_.y_m;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation = yaw_to_quaternion(pose_.yaw_rad);

  odometry.twist.twist.linear.x = vx;
  odometry.twist.twist.linear.y = vy;
  odometry.twist.twist.linear.z = 0.0;
  odometry.twist.twist.angular.x = 0.0;
  odometry.twist.twist.angular.y = 0.0;
  odometry.twist.twist.angular.z = wz;

  const double uncertainty_scale = 1.0 + 4.0 * (1.0 - std::clamp(quality_score, 0.0, 1.0));
  VOCovarianceConfig adaptive_covariance = covariance_config_;
  adaptive_covariance.position_variance *= uncertainty_scale;
  adaptive_covariance.yaw_variance *= uncertainty_scale;
  adaptive_covariance.linear_velocity_variance *= uncertainty_scale;
  adaptive_covariance.angular_velocity_variance *= uncertainty_scale;

  CovarianceBuilder covariance_builder(adaptive_covariance);
  odometry.pose.covariance = covariance_builder.pose_covariance();
  odometry.twist.covariance = covariance_builder.twist_covariance();
  return odometry;
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

void RGBDOdometryNode::publish_tracking_quality(const double score)
{
  Float32 quality;
  quality.data = static_cast<float>(std::clamp(score, 0.0, 1.0));
  tracking_quality_pub_->publish(quality);
}

}  // namespace savo_vo
