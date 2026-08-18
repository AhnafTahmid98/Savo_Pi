#pragma once

#include <string>
#include <vector>

#include "opencv2/core.hpp"

namespace savo_vo
{

struct RGBDCameraModel
{
  cv::Matx33d camera_matrix{cv::Matx33d::eye()};
  cv::Mat distortion_coefficients{};
  std::string distortion_model{};
  int width{0};
  int height{0};

  bool is_valid() const;
};

struct RGBDCorrespondences
{
  std::vector<cv::Point3f> previous_camera_points;
  std::vector<cv::Point2f> current_image_points;
};

struct PnPConfig
{
  int minimum_correspondences{30};
  int minimum_inliers{20};
  double minimum_inlier_ratio{0.50};
  int iterations{100};
  double reprojection_error_px{2.5};
  double confidence{0.99};
};

struct PnPEstimate
{
  bool success{false};
  int correspondence_count{0};
  int inlier_count{0};
  double inlier_ratio{0.0};

  // solvePnPRansac returns current_camera_T_previous_camera. Its inverse is
  // the actual current camera pose expressed in the previous camera frame.
  cv::Matx44d current_camera_T_previous_camera{cv::Matx44d::eye()};
  cv::Matx44d previous_camera_T_current_camera{cv::Matx44d::eye()};
  std::string rejection_reason{};
};

struct PlanarMotion
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

struct Pose2D
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

double depth_at_meters(
  const cv::Mat & aligned_depth,
  const cv::Point2f & color_pixel,
  double depth_scale_16u,
  double minimum_depth_m,
  double maximum_depth_m);

RGBDCorrespondences build_rgbd_correspondences(
  const std::vector<cv::Point2f> & previous_image_points,
  const std::vector<cv::Point2f> & current_image_points,
  const cv::Mat & previous_aligned_depth,
  const RGBDCameraModel & camera_model,
  double depth_scale_16u,
  double minimum_depth_m,
  double maximum_depth_m);

PnPEstimate estimate_current_camera_from_previous_camera(
  const RGBDCorrespondences & correspondences,
  const RGBDCameraModel & camera_model,
  const PnPConfig & config);

cv::Matx44d make_rigid_transform(
  const cv::Matx33d & rotation,
  const cv::Vec3d & translation);

cv::Matx44d make_rigid_transform_from_rpy(
  double x_m,
  double y_m,
  double z_m,
  double roll_rad,
  double pitch_rad,
  double yaw_rad);

cv::Matx44d invert_rigid_transform(const cv::Matx44d & transform);

cv::Matx44d base_increment_from_camera_increment(
  const cv::Matx44d & base_T_camera,
  const cv::Matx44d & previous_camera_T_current_camera);

bool rigid_transform_is_finite(const cv::Matx44d & transform);

double rigid_rotation_angle_rad(const cv::Matx44d & transform);

PlanarMotion planar_motion_from_transform(const cv::Matx44d & transform);

bool motion_is_within_limits(
  const PlanarMotion & motion,
  const cv::Matx44d & full_transform,
  double maximum_translation_m,
  double maximum_rotation_rad);

PlanarMotion suppress_vibration_motion(
  const PlanarMotion & motion,
  double translation_deadband_m,
  double rotation_deadband_rad);

Pose2D compose_pose(const Pose2D & pose, const PlanarMotion & body_increment);

}  // namespace savo_vo
