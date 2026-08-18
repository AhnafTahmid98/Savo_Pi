#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/calib3d.hpp"

#include "savo_vo/rgbd_geometry.hpp"
#include "savo_vo/tracking_quality.hpp"
#include "savo_vo/vo_constants.hpp"

namespace savo_vo
{
namespace
{

constexpr double kHalfPi = 1.5707963267948966;
constexpr double kTolerance = 1e-8;

cv::Matx44d robot_camera_extrinsic()
{
  return make_rigid_transform_from_rpy(
    0.130,
    0.0,
    0.225,
    -kHalfPi,
    0.0,
    -kHalfPi);
}

PlanarMotion round_trip_base_motion(const PlanarMotion & expected)
{
  const cv::Matx44d base_T_camera = robot_camera_extrinsic();
  const cv::Matx44d previous_base_T_current_base = make_rigid_transform_from_rpy(
    expected.x_m,
    expected.y_m,
    0.0,
    0.0,
    0.0,
    expected.yaw_rad);
  const cv::Matx44d previous_camera_T_current_camera =
    invert_rigid_transform(base_T_camera) *
    previous_base_T_current_base *
    base_T_camera;

  return planar_motion_from_transform(
    base_increment_from_camera_increment(
      base_T_camera,
      previous_camera_T_current_camera));
}

void expect_motion_near(
  const PlanarMotion & actual,
  const PlanarMotion & expected,
  const double tolerance = kTolerance)
{
  EXPECT_NEAR(actual.x_m, expected.x_m, tolerance);
  EXPECT_NEAR(actual.y_m, expected.y_m, tolerance);
  EXPECT_NEAR(actual.yaw_rad, expected.yaw_rad, tolerance);
}

TEST(RGBDGeometry, ForwardRobotMotionIsPositiveX)
{
  expect_motion_near(round_trip_base_motion({0.20, 0.0, 0.0}), {0.20, 0.0, 0.0});
}

TEST(RGBDGeometry, BackwardRobotMotionIsNegativeX)
{
  expect_motion_near(round_trip_base_motion({-0.20, 0.0, 0.0}), {-0.20, 0.0, 0.0});
}

TEST(RGBDGeometry, StrafeLeftIsPositiveY)
{
  expect_motion_near(round_trip_base_motion({0.0, 0.20, 0.0}), {0.0, 0.20, 0.0});
}

TEST(RGBDGeometry, StrafeRightIsNegativeY)
{
  expect_motion_near(round_trip_base_motion({0.0, -0.20, 0.0}), {0.0, -0.20, 0.0});
}

TEST(RGBDGeometry, CounterClockwiseYawIsPositive)
{
  expect_motion_near(round_trip_base_motion({0.0, 0.0, 0.20}), {0.0, 0.0, 0.20});
}

TEST(RGBDGeometry, ClockwiseYawIsNegative)
{
  expect_motion_near(round_trip_base_motion({0.0, 0.0, -0.20}), {0.0, 0.0, -0.20});
}

TEST(RGBDGeometry, CameraLeverArmIsRemovedForPureBaseYaw)
{
  const cv::Matx44d base_T_camera = robot_camera_extrinsic();
  const cv::Matx44d expected_base_increment = make_rigid_transform_from_rpy(
    0.0, 0.0, 0.0, 0.0, 0.0, 0.25);
  const cv::Matx44d camera_increment =
    invert_rigid_transform(base_T_camera) * expected_base_increment * base_T_camera;

  EXPECT_GT(
    std::hypot(camera_increment(0, 3), camera_increment(2, 3)),
    0.01);

  const PlanarMotion recovered = planar_motion_from_transform(
    base_increment_from_camera_increment(base_T_camera, camera_increment));
  expect_motion_near(recovered, {0.0, 0.0, 0.25});
}

TEST(RGBDGeometry, PnPDirectionIsInvertedToActualForwardCameraMotion)
{
  RGBDCameraModel model;
  model.camera_matrix = cv::Matx33d(
    600.0, 0.0, 320.0,
    0.0, 600.0, 240.0,
    0.0, 0.0, 1.0);
  model.distortion_model = "plumb_bob";
  model.width = 640;
  model.height = 480;

  RGBDCorrespondences correspondences;
  for (int row = -2; row <= 2; ++row) {
    for (int column = -3; column <= 3; ++column) {
      correspondences.previous_camera_points.emplace_back(
        0.12F * static_cast<float>(column),
        0.10F * static_cast<float>(row),
        2.0F + 0.03F * static_cast<float>(row + column + 5));
    }
  }

  const cv::Matx44d previous_camera_T_current_camera =
    make_rigid_transform_from_rpy(0.0, 0.0, 0.10, 0.0, 0.0, 0.0);
  const cv::Matx44d current_camera_T_previous_camera =
    invert_rigid_transform(previous_camera_T_current_camera);

  cv::Matx33d current_rotation(
    current_camera_T_previous_camera(0, 0),
    current_camera_T_previous_camera(0, 1),
    current_camera_T_previous_camera(0, 2),
    current_camera_T_previous_camera(1, 0),
    current_camera_T_previous_camera(1, 1),
    current_camera_T_previous_camera(1, 2),
    current_camera_T_previous_camera(2, 0),
    current_camera_T_previous_camera(2, 1),
    current_camera_T_previous_camera(2, 2));
  cv::Vec3d rotation_vector;
  cv::Rodrigues(current_rotation, rotation_vector);
  const cv::Vec3d translation_vector(
    current_camera_T_previous_camera(0, 3),
    current_camera_T_previous_camera(1, 3),
    current_camera_T_previous_camera(2, 3));

  cv::projectPoints(
    correspondences.previous_camera_points,
    rotation_vector,
    translation_vector,
    model.camera_matrix,
    model.distortion_coefficients,
    correspondences.current_image_points);

  PnPConfig config;
  config.minimum_correspondences = 12;
  config.minimum_inliers = 10;
  config.minimum_inlier_ratio = 0.90;
  config.reprojection_error_px = 0.5;

  const PnPEstimate estimate = estimate_current_camera_from_previous_camera(
    correspondences,
    model,
    config);

  ASSERT_TRUE(estimate.success) << estimate.rejection_reason;
  EXPECT_NEAR(estimate.previous_camera_T_current_camera(2, 3), 0.10, 1e-3);

  const PlanarMotion base_motion = planar_motion_from_transform(
    base_increment_from_camera_increment(
      robot_camera_extrinsic(),
      estimate.previous_camera_T_current_camera));
  EXPECT_NEAR(base_motion.x_m, 0.10, 1e-3);
  EXPECT_NEAR(base_motion.y_m, 0.0, 1e-3);
  EXPECT_NEAR(base_motion.yaw_rad, 0.0, 1e-3);
}

TEST(RGBDGeometry, InvalidDepthSamplesAreIgnored)
{
  RGBDCameraModel model;
  model.camera_matrix = cv::Matx33d(
    100.0, 0.0, 1.0,
    0.0, 100.0, 1.0,
    0.0, 0.0, 1.0);
  model.distortion_model = "plumb_bob";
  model.width = 4;
  model.height = 1;

  cv::Mat depth(1, 4, CV_32FC1);
  depth.at<float>(0, 0) = 0.0F;
  depth.at<float>(0, 1) = std::numeric_limits<float>::quiet_NaN();
  depth.at<float>(0, 2) = 9.0F;
  depth.at<float>(0, 3) = 1.5F;

  const std::vector<cv::Point2f> previous{
    {0.0F, 0.0F}, {1.0F, 0.0F}, {2.0F, 0.0F}, {3.0F, 0.0F}};
  const std::vector<cv::Point2f> current = previous;
  const RGBDCorrespondences correspondences = build_rgbd_correspondences(
    previous,
    current,
    depth,
    model,
    0.001,
    0.20,
    6.0);

  ASSERT_EQ(correspondences.previous_camera_points.size(), 1U);
  EXPECT_NEAR(correspondences.previous_camera_points.front().z, 1.5, 1e-6);
}

TEST(RGBDGeometry, UnsupportedDistortionModelIsRejected)
{
  RGBDCameraModel model;
  model.camera_matrix = cv::Matx33d(
    600.0, 0.0, 320.0,
    0.0, 600.0, 240.0,
    0.0, 0.0, 1.0);
  model.distortion_model = "equidistant";
  model.width = 640;
  model.height = 480;

  EXPECT_FALSE(model.is_valid());
}

TEST(RGBDGeometry, SixteenBitDepthUsesConfiguredMillimeterScale)
{
  cv::Mat depth(1, 1, CV_16UC1, cv::Scalar(1250));
  EXPECT_NEAR(
    depth_at_meters(depth, {0.0F, 0.0F}, 0.001, 0.20, 6.0),
    1.25,
    1e-9);
}

TEST(RGBDGeometry, ExcessiveMotionIsRejectedRatherThanClamped)
{
  const cv::Matx44d excessive = make_rigid_transform_from_rpy(
    0.50, 0.0, 0.0, 0.0, 0.0, 0.0);
  const PlanarMotion motion = planar_motion_from_transform(excessive);

  EXPECT_FALSE(motion_is_within_limits(motion, excessive, 0.15, 0.20));
  EXPECT_DOUBLE_EQ(motion.x_m, 0.50);
}

TEST(RGBDGeometry, RepeatedStationaryMotionDoesNotDrift)
{
  Pose2D pose;
  for (int iteration = 0; iteration < 10000; ++iteration) {
    const PlanarMotion optical_noise{0.0002, -0.0001, 0.0002};
    pose = compose_pose(
      pose,
      suppress_vibration_motion(optical_noise, 0.001, 0.001));
  }

  EXPECT_DOUBLE_EQ(pose.x_m, 0.0);
  EXPECT_DOUBLE_EQ(pose.y_m, 0.0);
  EXPECT_DOUBLE_EQ(pose.yaw_rad, 0.0);
}

TEST(RGBDGeometry, PublicOdometryUsesEkfBaseFootprintContract)
{
  EXPECT_STREQ(constants::kOdomFrame, "odom");
  EXPECT_STREQ(constants::kBaseFrame, "base_footprint");
  EXPECT_FALSE(constants::kDefaultPublishTf);
}

TEST(RGBDGeometry, TrackingQualityRequiresGeometricInliers)
{
  EXPECT_DOUBLE_EQ(compute_tracking_score(300, 280, 240, 0, 300), 0.0);
  EXPECT_GT(compute_tracking_score(300, 280, 240, 220, 300), 0.80);
}

}  // namespace
}  // namespace savo_vo
