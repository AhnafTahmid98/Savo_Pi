#include "savo_vo/rgbd_odometry_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include "savo_vo/geometry_utils.hpp"
#include "savo_vo/vo_constants.hpp"

namespace savo_vo
{
namespace
{

double median_value(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }

  const auto middle = values.begin() + static_cast<long>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

bool is_finite_positive(const double value)
{
  return std::isfinite(value) && value > 0.0;
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

  publish_status("waiting for RGB-D input");
  publish_tracking_quality(0.0);

  RCLCPP_INFO(
    get_logger(),
    "RGB-D odometry node started: color=%s, depth=%s, odom=%s",
    color_image_topic_.c_str(),
    depth_image_topic_.c_str(),
    odom_topic_.c_str());
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
  tracking_quality_topic_ = get_parameter(constants::kTrackingQualityTopicParam).as_string();

  odom_frame_ = get_parameter(constants::kOdomFrameParam).as_string();
  base_frame_ = get_parameter(constants::kBaseFrameParam).as_string();
  camera_frame_ = get_parameter(constants::kCameraFrameParam).as_string();

  max_image_delay_s_ = std::max(0.01, get_parameter("max_image_delay_s").as_double());
  max_depth_delay_s_ = std::max(0.01, get_parameter("max_depth_delay_s").as_double());

  min_features_ = std::max(1, static_cast<int>(get_parameter("min_features").as_int()));
  good_features_target_ = std::max(
    1,
    static_cast<int>(get_parameter("good_features_target").as_int()));
  max_features_ = std::max(
    min_features_,
    static_cast<int>(get_parameter("max_features").as_int()));

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

void RGBDOdometryNode::create_timers()
{
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RGBDOdometryNode::publish_waiting_status, this));
}

void RGBDOdometryNode::publish_waiting_status()
{
  if (has_previous_frame_) {
    return;
  }

  if (!has_required_inputs()) {
    publish_status("waiting for RGB-D input");
    publish_tracking_quality(0.0);
    return;
  }

  publish_status("waiting_for_reference: waiting for second RGB-D frame");
  publish_tracking_quality(0.0);
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
}

void RGBDOdometryNode::on_camera_info(const CameraInfo::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  latest_camera_info_ = msg;
}

void RGBDOdometryNode::try_process_frame()
{
  if (!has_required_inputs()) {
    publish_status("waiting for RGB-D input");
    publish_tracking_quality(0.0);
    return;
  }

  const auto intrinsics = camera_intrinsics_from_info(*latest_camera_info_);

  if (!intrinsics.is_valid()) {
    publish_status("error: invalid camera intrinsics");
    publish_tracking_quality(0.0);
    return;
  }

  cv::Mat gray_image;
  cv::Mat depth_image;

  if (!convert_latest_images(gray_image, depth_image)) {
    publish_status("error: failed to convert RGB-D images");
    publish_tracking_quality(0.0);
    return;
  }

  Odometry odometry;
  const auto quality = estimate_visual_motion(
    gray_image,
    depth_image,
    intrinsics,
    odometry);

  publish_tracking_quality(quality.score);
  publish_status(quality.message);

  if (quality.is_good(min_tracking_quality_, min_features_)) {
    publish_odometry(odometry);
  }
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

bool RGBDOdometryNode::convert_latest_images(
  cv::Mat & gray_image,
  cv::Mat & depth_image) const
{
  try {
    const auto color_cv = cv_bridge::toCvShare(latest_color_);
    const auto depth_cv = cv_bridge::toCvShare(latest_depth_);

    const cv::Mat & color = color_cv->image;

    if (color.empty() || depth_cv->image.empty()) {
      return false;
    }

    if (color.channels() == 1) {
      gray_image = color.clone();
    } else if (color.channels() == 3) {
      cv::cvtColor(color, gray_image, cv::COLOR_BGR2GRAY);
    } else if (color.channels() == 4) {
      cv::cvtColor(color, gray_image, cv::COLOR_BGRA2GRAY);
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
  const cv::Mat & depth_image,
  const CameraIntrinsics & intrinsics,
  Odometry & odometry)
{
  const auto current_stamp = rclcpp::Time(latest_color_->header.stamp);
  const double current_stamp_s = current_stamp.seconds();

  if (!has_previous_frame_) {
    previous_features_ = detect_reference_features(gray_image);
    previous_gray_image_ = gray_image.clone();
    previous_stamp_s_ = current_stamp_s;
    has_previous_frame_ = true;

    TrackingQuality quality;
    quality.feature_count = static_cast<int>(previous_features_.size());
    quality.tracked_count = 0;
    quality.score = 0.0;
    quality.state = VOTrackingState::kWaitingForReference;
    quality.message = "waiting_for_reference: first RGB-D frame stored";
    return quality;
  }

  if (previous_features_.size() < static_cast<std::size_t>(min_features_)) {
    previous_features_ = detect_reference_features(previous_gray_image_);
  }

  if (previous_features_.empty()) {
    has_previous_frame_ = false;

    TrackingQuality quality;
    quality.state = VOTrackingState::kRejected;
    quality.message = "rejected: no reference features";
    return quality;
  }

  std::vector<cv::Point2f> current_features;
  std::vector<unsigned char> status;
  std::vector<float> errors;

  cv::calcOpticalFlowPyrLK(
    previous_gray_image_,
    gray_image,
    previous_features_,
    current_features,
    status,
    errors);

  std::vector<cv::Point2f> previous_good;
  std::vector<cv::Point2f> current_good;

  previous_good.reserve(previous_features_.size());
  current_good.reserve(current_features.size());

  for (std::size_t i = 0; i < status.size(); ++i) {
    if (!status[i]) {
      continue;
    }

    const auto & prev = previous_features_[i];
    const auto & curr = current_features[i];

    if (curr.x < 0.0F || curr.y < 0.0F ||
        curr.x >= static_cast<float>(gray_image.cols) ||
        curr.y >= static_cast<float>(gray_image.rows))
    {
      continue;
    }

    previous_good.push_back(prev);
    current_good.push_back(curr);
  }

  TrackingQualityEstimator estimator(
    min_features_,
    good_features_target_,
    min_tracking_quality_);

  auto quality = estimator.evaluate(
    static_cast<int>(previous_features_.size()),
    static_cast<int>(current_good.size()));

  if (!quality.is_good(min_tracking_quality_, min_features_)) {
    previous_features_ = detect_reference_features(gray_image);
    previous_gray_image_ = gray_image.clone();
    previous_stamp_s_ = current_stamp_s;
    return quality;
  }

  cv::Mat inlier_mask;
  const cv::Mat affine = cv::estimateAffinePartial2D(
    previous_good,
    current_good,
    inlier_mask,
    cv::RANSAC,
    3.0);

  if (affine.empty()) {
    previous_features_ = detect_reference_features(gray_image);
    previous_gray_image_ = gray_image.clone();
    previous_stamp_s_ = current_stamp_s;

    quality.state = VOTrackingState::kLost;
    quality.score = 0.0;
    quality.message = "lost: affine motion estimate failed";
    return quality;
  }

  const double dx_px = affine.at<double>(0, 2);
  const double dy_px = affine.at<double>(1, 2);
  const double yaw_delta = normalize_angle(
    std::atan2(affine.at<double>(1, 0), affine.at<double>(0, 0)));

  const double median_depth_m = estimate_median_depth_m(depth_image, current_good);

  if (!is_finite_positive(median_depth_m)) {
    previous_features_ = detect_reference_features(gray_image);
    previous_gray_image_ = gray_image.clone();
    previous_stamp_s_ = current_stamp_s;

    quality.state = VOTrackingState::kRejected;
    quality.score = 0.0;
    quality.message = "rejected: no valid depth for tracked features";
    return quality;
  }

  double dt_s = current_stamp_s - previous_stamp_s_;
  if (dt_s <= 0.001 || !std::isfinite(dt_s)) {
    dt_s = 1.0 / 30.0;
  }

  double delta_x_m = -(dx_px / intrinsics.fx) * median_depth_m;
  double delta_y_m = -(dy_px / intrinsics.fy) * median_depth_m;

  delta_x_m = std::clamp(delta_x_m, -max_translation_jump_m_, max_translation_jump_m_);
  delta_y_m = std::clamp(delta_y_m, -max_translation_jump_m_, max_translation_jump_m_);

  const double delta_yaw_rad = std::clamp(
    yaw_delta,
    -max_rotation_jump_rad_,
    max_rotation_jump_rad_);

  const double cos_yaw = std::cos(yaw_rad_);
  const double sin_yaw = std::sin(yaw_rad_);

  const double raw_world_dx_m = cos_yaw * delta_x_m - sin_yaw * delta_y_m;
  const double raw_world_dy_m = sin_yaw * delta_x_m + cos_yaw * delta_y_m;
  const double raw_dyaw_rad = delta_yaw_rad;

  constexpr double kVibrationTranslationDeadbandM = 0.003;
  constexpr double kVibrationYawDeadbandRad = 0.003;

  const double raw_translation_m = std::hypot(raw_world_dx_m, raw_world_dy_m);
  const bool suppress_vibration_motion =
    raw_translation_m < kVibrationTranslationDeadbandM &&
    std::abs(raw_dyaw_rad) < kVibrationYawDeadbandRad;

  const double filtered_world_dx_m = suppress_vibration_motion ? 0.0 : raw_world_dx_m;
  const double filtered_world_dy_m = suppress_vibration_motion ? 0.0 : raw_world_dy_m;
  const double filtered_dyaw_rad = suppress_vibration_motion ? 0.0 : raw_dyaw_rad;

  pose_x_m_ += filtered_world_dx_m;
  pose_y_m_ += filtered_world_dy_m;
  yaw_rad_ = normalize_angle(yaw_rad_ + filtered_dyaw_rad);

  const double vx = delta_x_m / dt_s;
  const double vy = delta_y_m / dt_s;
  const double wz = delta_yaw_rad / dt_s;

  odometry = build_odometry_message(vx, vy, wz, current_stamp);

  previous_features_ = detect_reference_features(gray_image);
  previous_gray_image_ = gray_image.clone();
  previous_stamp_s_ = current_stamp_s;

  quality.state = VOTrackingState::kTracking;
  quality.message = build_tracking_message(
    quality.state,
    quality.feature_count,
    quality.tracked_count,
    quality.score);

  return quality;
}

std::vector<cv::Point2f> RGBDOdometryNode::detect_reference_features(
  const cv::Mat & gray_image) const
{
  std::vector<cv::Point2f> features;

  if (gray_image.empty()) {
    return features;
  }

  cv::goodFeaturesToTrack(
    gray_image,
    features,
    max_features_,
    0.01,
    8.0);

  return features;
}

double RGBDOdometryNode::estimate_median_depth_m(
  const cv::Mat & depth_image,
  const std::vector<cv::Point2f> & points) const
{
  std::vector<double> depths_m;
  depths_m.reserve(points.size());

  for (const auto & point : points) {
    const int x = static_cast<int>(std::round(point.x));
    const int y = static_cast<int>(std::round(point.y));

    if (x < 0 || y < 0 || x >= depth_image.cols || y >= depth_image.rows) {
      continue;
    }

    double depth_m = 0.0;

    if (depth_image.type() == CV_16UC1) {
      depth_m = static_cast<double>(depth_image.at<std::uint16_t>(y, x)) * 0.001;
    } else if (depth_image.type() == CV_32FC1) {
      depth_m = static_cast<double>(depth_image.at<float>(y, x));
    } else {
      continue;
    }

    if (is_finite_positive(depth_m) && depth_m < 8.0) {
      depths_m.push_back(depth_m);
    }
  }

  return median_value(depths_m);
}

RGBDOdometryNode::Odometry RGBDOdometryNode::build_odometry_message(
  const double vx,
  const double vy,
  const double wz,
  const rclcpp::Time & stamp)
{
  Odometry odometry;

  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_;
  odometry.child_frame_id = base_frame_;

  odometry.pose.pose.position.x = pose_x_m_;
  odometry.pose.pose.position.y = pose_y_m_;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation = yaw_to_quaternion(yaw_rad_);

  odometry.twist.twist.linear.x = vx;
  odometry.twist.twist.linear.y = vy;
  odometry.twist.twist.linear.z = 0.0;
  odometry.twist.twist.angular.z = wz;

  CovarianceBuilder covariance_builder(covariance_config_);
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
