#include "savo_vo/rgbd_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "opencv2/calib3d.hpp"

#include "savo_vo/geometry_utils.hpp"

namespace savo_vo
{
namespace
{

bool finite_value(const double value)
{
  return std::isfinite(value);
}

cv::Matx33d rotation_from_rpy(
  const double roll_rad,
  const double pitch_rad,
  const double yaw_rad)
{
  const double cr = std::cos(roll_rad);
  const double sr = std::sin(roll_rad);
  const double cp = std::cos(pitch_rad);
  const double sp = std::sin(pitch_rad);
  const double cy = std::cos(yaw_rad);
  const double sy = std::sin(yaw_rad);

  const cv::Matx33d rotation_x(
    1.0, 0.0, 0.0,
    0.0, cr, -sr,
    0.0, sr, cr);

  const cv::Matx33d rotation_y(
    cp, 0.0, sp,
    0.0, 1.0, 0.0,
    -sp, 0.0, cp);

  const cv::Matx33d rotation_z(
    cy, -sy, 0.0,
    sy, cy, 0.0,
    0.0, 0.0, 1.0);

  return rotation_z * rotation_y * rotation_x;
}

cv::Matx33d rotation_block(const cv::Matx44d & transform)
{
  return cv::Matx33d(
    transform(0, 0), transform(0, 1), transform(0, 2),
    transform(1, 0), transform(1, 1), transform(1, 2),
    transform(2, 0), transform(2, 1), transform(2, 2));
}

cv::Vec3d translation_block(const cv::Matx44d & transform)
{
  return cv::Vec3d(transform(0, 3), transform(1, 3), transform(2, 3));
}

}  // namespace

bool RGBDCameraModel::is_valid() const
{
  const double fx = camera_matrix(0, 0);
  const double fy = camera_matrix(1, 1);
  const double cx = camera_matrix(0, 2);
  const double cy = camera_matrix(1, 2);

  const bool supported_distortion =
    distortion_model.empty() ||
    distortion_model == "none" ||
    distortion_model == "plumb_bob" ||
    distortion_model == "rational_polynomial";

  bool finite_distortion = true;
  if (!distortion_coefficients.empty()) {
    finite_distortion = distortion_coefficients.depth() == CV_64F;
    if (finite_distortion) {
      const cv::Mat flattened = distortion_coefficients.reshape(1, 1);
      for (int column = 0; column < flattened.cols; ++column) {
        finite_distortion = finite_distortion &&
          finite_value(flattened.at<double>(0, column));
      }
    }
  }

  return supported_distortion &&
         finite_distortion &&
         finite_value(fx) && fx > 0.0 &&
         finite_value(fy) && fy > 0.0 &&
         finite_value(cx) && finite_value(cy) &&
         width > 0 && height > 0;
}

double depth_at_meters(
  const cv::Mat & aligned_depth,
  const cv::Point2f & color_pixel,
  const double depth_scale_16u,
  const double minimum_depth_m,
  const double maximum_depth_m)
{
  if (aligned_depth.empty() ||
    !finite_value(color_pixel.x) || !finite_value(color_pixel.y))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const int x = static_cast<int>(std::lround(color_pixel.x));
  const int y = static_cast<int>(std::lround(color_pixel.y));

  if (x < 0 || y < 0 || x >= aligned_depth.cols || y >= aligned_depth.rows) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double depth_m = std::numeric_limits<double>::quiet_NaN();

  if (aligned_depth.type() == CV_16UC1) {
    const auto raw_depth = aligned_depth.at<std::uint16_t>(y, x);
    depth_m = static_cast<double>(raw_depth) * depth_scale_16u;
  } else if (aligned_depth.type() == CV_32FC1) {
    depth_m = static_cast<double>(aligned_depth.at<float>(y, x));
  }

  if (!finite_value(depth_m) ||
    depth_m < minimum_depth_m || depth_m > maximum_depth_m)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return depth_m;
}

RGBDCorrespondences build_rgbd_correspondences(
  const std::vector<cv::Point2f> & previous_image_points,
  const std::vector<cv::Point2f> & current_image_points,
  const cv::Mat & previous_aligned_depth,
  const RGBDCameraModel & camera_model,
  const double depth_scale_16u,
  const double minimum_depth_m,
  const double maximum_depth_m)
{
  RGBDCorrespondences correspondences;

  if (!camera_model.is_valid() ||
    previous_image_points.size() != current_image_points.size())
  {
    return correspondences;
  }

  std::vector<cv::Point2f> valid_previous_pixels;
  std::vector<cv::Point2f> valid_current_pixels;
  std::vector<double> valid_depths_m;

  valid_previous_pixels.reserve(previous_image_points.size());
  valid_current_pixels.reserve(current_image_points.size());
  valid_depths_m.reserve(previous_image_points.size());

  for (std::size_t index = 0; index < previous_image_points.size(); ++index) {
    const double depth_m = depth_at_meters(
      previous_aligned_depth,
      previous_image_points[index],
      depth_scale_16u,
      minimum_depth_m,
      maximum_depth_m);

    if (!finite_value(depth_m)) {
      continue;
    }

    valid_previous_pixels.push_back(previous_image_points[index]);
    valid_current_pixels.push_back(current_image_points[index]);
    valid_depths_m.push_back(depth_m);
  }

  if (valid_previous_pixels.empty()) {
    return correspondences;
  }

  std::vector<cv::Point2f> normalized_previous_points;
  cv::undistortPoints(
    valid_previous_pixels,
    normalized_previous_points,
    camera_model.camera_matrix,
    camera_model.distortion_coefficients);

  correspondences.previous_camera_points.reserve(normalized_previous_points.size());
  correspondences.current_image_points.reserve(normalized_previous_points.size());

  for (std::size_t index = 0; index < normalized_previous_points.size(); ++index) {
    const double depth_m = valid_depths_m[index];
    const auto & normalized = normalized_previous_points[index];

    correspondences.previous_camera_points.emplace_back(
      static_cast<float>(normalized.x * depth_m),
      static_cast<float>(normalized.y * depth_m),
      static_cast<float>(depth_m));
    correspondences.current_image_points.push_back(valid_current_pixels[index]);
  }

  return correspondences;
}

PnPEstimate estimate_current_camera_from_previous_camera(
  const RGBDCorrespondences & correspondences,
  const RGBDCameraModel & camera_model,
  const PnPConfig & config)
{
  PnPEstimate estimate;
  estimate.correspondence_count = static_cast<int>(
    correspondences.previous_camera_points.size());

  if (!camera_model.is_valid()) {
    estimate.rejection_reason = "invalid camera model";
    return estimate;
  }

  if (correspondences.previous_camera_points.size() !=
    correspondences.current_image_points.size())
  {
    estimate.rejection_reason = "3-D/2-D correspondence size mismatch";
    return estimate;
  }

  if (estimate.correspondence_count < std::max(4, config.minimum_correspondences)) {
    estimate.rejection_reason = "insufficient valid depth correspondences";
    return estimate;
  }

  cv::Mat rotation_vector;
  cv::Mat translation_vector;
  cv::Mat inlier_indices;

  bool solved = false;
  try {
    solved = cv::solvePnPRansac(
      correspondences.previous_camera_points,
      correspondences.current_image_points,
      camera_model.camera_matrix,
      camera_model.distortion_coefficients,
      rotation_vector,
      translation_vector,
      false,
      std::max(1, config.iterations),
      static_cast<float>(std::max(0.1, config.reprojection_error_px)),
      std::clamp(config.confidence, 0.01, 0.9999),
      inlier_indices,
      cv::SOLVEPNP_EPNP);
  } catch (const cv::Exception &) {
    estimate.rejection_reason = "PnP raised an OpenCV exception";
    return estimate;
  }

  if (!solved || rotation_vector.empty() || translation_vector.empty()) {
    estimate.rejection_reason = "PnP failed";
    return estimate;
  }

  estimate.inlier_count = static_cast<int>(inlier_indices.total());
  estimate.inlier_ratio = estimate.correspondence_count > 0 ?
    static_cast<double>(estimate.inlier_count) /
    static_cast<double>(estimate.correspondence_count) : 0.0;

  if (estimate.inlier_count < std::max(4, config.minimum_inliers)) {
    estimate.rejection_reason = "insufficient PnP inliers";
    return estimate;
  }

  if (estimate.inlier_ratio < std::clamp(config.minimum_inlier_ratio, 0.0, 1.0)) {
    estimate.rejection_reason = "PnP inlier ratio below threshold";
    return estimate;
  }

  cv::Mat rotation_matrix;
  cv::Rodrigues(rotation_vector, rotation_matrix);

  cv::Matx33d rotation;
  cv::Vec3d translation;
  for (int row = 0; row < 3; ++row) {
    translation(row) = translation_vector.at<double>(row, 0);
    for (int column = 0; column < 3; ++column) {
      rotation(row, column) = rotation_matrix.at<double>(row, column);
    }
  }

  estimate.current_camera_T_previous_camera = make_rigid_transform(
    rotation,
    translation);
  estimate.previous_camera_T_current_camera = invert_rigid_transform(
    estimate.current_camera_T_previous_camera);

  if (!rigid_transform_is_finite(estimate.current_camera_T_previous_camera) ||
    !rigid_transform_is_finite(estimate.previous_camera_T_current_camera))
  {
    estimate.rejection_reason = "PnP transform is not finite";
    return estimate;
  }

  estimate.success = true;
  return estimate;
}

cv::Matx44d make_rigid_transform(
  const cv::Matx33d & rotation,
  const cv::Vec3d & translation)
{
  cv::Matx44d transform = cv::Matx44d::eye();

  for (int row = 0; row < 3; ++row) {
    transform(row, 3) = translation(row);
    for (int column = 0; column < 3; ++column) {
      transform(row, column) = rotation(row, column);
    }
  }

  return transform;
}

cv::Matx44d make_rigid_transform_from_rpy(
  const double x_m,
  const double y_m,
  const double z_m,
  const double roll_rad,
  const double pitch_rad,
  const double yaw_rad)
{
  return make_rigid_transform(
    rotation_from_rpy(roll_rad, pitch_rad, yaw_rad),
    cv::Vec3d(x_m, y_m, z_m));
}

cv::Matx44d invert_rigid_transform(const cv::Matx44d & transform)
{
  const cv::Matx33d rotation = rotation_block(transform);
  const cv::Matx33d inverse_rotation = rotation.t();
  const cv::Vec3d inverse_translation = -inverse_rotation * translation_block(transform);
  return make_rigid_transform(inverse_rotation, inverse_translation);
}

cv::Matx44d base_increment_from_camera_increment(
  const cv::Matx44d & base_T_camera,
  const cv::Matx44d & previous_camera_T_current_camera)
{
  const cv::Matx44d camera_T_base = invert_rigid_transform(base_T_camera);
  return base_T_camera * previous_camera_T_current_camera * camera_T_base;
}

bool rigid_transform_is_finite(const cv::Matx44d & transform)
{
  for (const double value : transform.val) {
    if (!finite_value(value)) {
      return false;
    }
  }
  return true;
}

double rigid_rotation_angle_rad(const cv::Matx44d & transform)
{
  cv::Vec3d rotation_vector;
  cv::Rodrigues(rotation_block(transform), rotation_vector);
  return cv::norm(rotation_vector);
}

PlanarMotion planar_motion_from_transform(const cv::Matx44d & transform)
{
  PlanarMotion motion;
  motion.x_m = transform(0, 3);
  motion.y_m = transform(1, 3);
  motion.yaw_rad = normalize_angle(std::atan2(transform(1, 0), transform(0, 0)));
  return motion;
}

bool motion_is_within_limits(
  const PlanarMotion & motion,
  const cv::Matx44d & full_transform,
  const double maximum_translation_m,
  const double maximum_rotation_rad)
{
  if (!rigid_transform_is_finite(full_transform) ||
    !finite_value(motion.x_m) || !finite_value(motion.y_m) ||
    !finite_value(motion.yaw_rad))
  {
    return false;
  }

  const cv::Vec3d translation = translation_block(full_transform);
  const double translation_norm_m = cv::norm(translation);
  const double rotation_angle_rad = rigid_rotation_angle_rad(full_transform);

  return finite_value(translation_norm_m) && finite_value(rotation_angle_rad) &&
         translation_norm_m <= maximum_translation_m &&
         rotation_angle_rad <= maximum_rotation_rad;
}

PlanarMotion suppress_vibration_motion(
  const PlanarMotion & motion,
  const double translation_deadband_m,
  const double rotation_deadband_rad)
{
  const bool below_both_deadbands =
    std::hypot(motion.x_m, motion.y_m) < std::max(0.0, translation_deadband_m) &&
    std::abs(motion.yaw_rad) < std::max(0.0, rotation_deadband_rad);
  return below_both_deadbands ? PlanarMotion{} : motion;
}

Pose2D compose_pose(const Pose2D & pose, const PlanarMotion & body_increment)
{
  const double cos_yaw = std::cos(pose.yaw_rad);
  const double sin_yaw = std::sin(pose.yaw_rad);

  Pose2D composed;
  composed.x_m = pose.x_m +
    cos_yaw * body_increment.x_m - sin_yaw * body_increment.y_m;
  composed.y_m = pose.y_m +
    sin_yaw * body_increment.x_m + cos_yaw * body_increment.y_m;
  composed.yaw_rad = normalize_angle(pose.yaw_rad + body_increment.yaw_rad);
  return composed;
}

}  // namespace savo_vo
