#pragma once

#include <memory>
#include <string>
#include <vector>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_vo/covariance_builder.hpp"
#include "savo_vo/latest_frame_selector.hpp"
#include "savo_vo/rgbd_geometry.hpp"
#include "savo_vo/tracking_quality.hpp"
#include "savo_vo/vo_types.hpp"

namespace savo_vo
{

class RGBDOdometryNode final : public rclcpp::Node
{
public:
  explicit RGBDOdometryNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Float32 = std_msgs::msg::Float32;
  using Image = sensor_msgs::msg::Image;
  using Odometry = nav_msgs::msg::Odometry;
  using String = std_msgs::msg::String;
  using ImageSyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  static rclcpp::QoS camera_qos();
  static rclcpp::QoS odometry_qos();
  static rclcpp::QoS status_qos();

  void declare_parameters();
  void load_parameters();
  void create_publishers();
  void create_subscribers();
  void create_timers();

  void on_synchronized_images(
    const Image::ConstSharedPtr & color_msg,
    const Image::ConstSharedPtr & aligned_depth_msg);
  void process_latest_images();
  void on_camera_info(const CameraInfo::SharedPtr msg);
  void publish_waiting_status();

  RGBDCameraModel camera_model_from_info(const CameraInfo & info) const;

  bool convert_images(
    const Image::ConstSharedPtr & color_msg,
    const Image::ConstSharedPtr & depth_msg,
    cv::Mat & gray_image,
    cv::Mat & depth_image) const;

  TrackingQuality estimate_visual_motion(
    const cv::Mat & gray_image,
    const cv::Mat & aligned_depth_image,
    const RGBDCameraModel & camera_model,
    const rclcpp::Time & stamp,
    Odometry & odometry);

  void reset_reference(
    const cv::Mat & gray_image,
    const cv::Mat & aligned_depth_image,
    double stamp_s);

  std::vector<cv::Point2f> detect_reference_features(
    const cv::Mat & gray_image) const;

  Odometry build_odometry_message(
    double vx,
    double vy,
    double wz,
    double quality_score,
    const rclcpp::Time & stamp);

  void publish_odometry(const Odometry & odometry);
  void publish_status(const std::string & message);
  void publish_tracking_quality(double score);

  std::string color_image_topic_;
  std::string color_camera_info_topic_;
  std::string depth_image_topic_;

  std::string odom_topic_;
  std::string status_topic_;
  std::string tracking_quality_topic_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string camera_frame_;

  int sync_queue_size_{2};
  double processing_rate_hz_{15.0};
  double max_sync_delta_s_{0.02};
  double max_frame_interval_s_{0.20};

  int min_features_{80};
  int good_features_target_{300};
  int max_features_{800};

  double min_tracking_quality_{0.35};
  int min_depth_correspondences_{30};
  int min_pnp_inliers_{20};
  double min_pnp_inlier_ratio_{0.50};
  int pnp_iterations_{100};
  double pnp_reprojection_error_px_{2.5};
  double pnp_confidence_{0.99};

  double depth_scale_16u_{0.001};
  double min_depth_m_{0.20};
  double max_depth_m_{6.0};

  double max_translation_jump_m_{0.15};
  double max_rotation_jump_rad_{0.20};
  double translation_deadband_m_{0.001};
  double rotation_deadband_rad_{0.001};

  bool publish_tf_{false};
  bool publish_diagnostics_{true};

  Pose2D pose_{};
  double previous_stamp_s_{0.0};
  bool has_previous_frame_{false};

  cv::Mat previous_gray_image_;
  cv::Mat previous_aligned_depth_image_;
  std::vector<cv::Point2f> previous_features_;

  cv::Matx44d base_T_camera_{cv::Matx44d::eye()};
  VOCovarianceConfig covariance_config_;

  CameraInfo::SharedPtr latest_camera_info_;
  LatestFrameSelector latest_frame_selector_;
  Image::ConstSharedPtr pending_color_image_;
  Image::ConstSharedPtr pending_aligned_depth_image_;

  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<String>::SharedPtr status_pub_;
  rclcpp::Publisher<Float32>::SharedPtr tracking_quality_pub_;

  message_filters::Subscriber<Image> color_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> image_sync_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
};

}  // namespace savo_vo
