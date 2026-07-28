#ifndef SAVO_MAPPING__TF_POSE_READER_HPP_
#define SAVO_MAPPING__TF_POSE_READER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "savo_mapping/slam_contract.hpp"
#include "tf2_ros/buffer.hpp"

namespace savo_mapping
{

struct TfPoseReaderOptions
{
  std::string target_frame{slam::MAP_FRAME};
  std::string source_frame{slam::BASE_FRAME};
  double lookup_timeout_sec{0.20};
  double stale_timeout_sec{1.00};
};

[[nodiscard]] std::string validate_tf_pose_reader_options(
  const TfPoseReaderOptions & options);

struct TfPoseSnapshot
{
  bool valid{false};
  bool fresh{false};
  std::string target_frame;
  std::string source_frame;
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double quaternion_x{0.0};
  double quaternion_y{0.0};
  double quaternion_z{0.0};
  double quaternion_w{1.0};
  double yaw_rad{0.0};
  rclcpp::Time transform_stamp{0, 0, RCL_ROS_TIME};
  double age_sec{0.0};
  double lookup_duration_sec{0.0};
  std::string reason{"tf_pose_transform_unavailable"};
  std::string detail;
};

class TfPoseReader final
{
public:
  TfPoseReader(
    rclcpp::Clock::SharedPtr clock,
    tf2_ros::Buffer::SharedPtr buffer);

  TfPoseReader(
    rclcpp::Clock::SharedPtr clock,
    tf2_ros::Buffer::SharedPtr buffer,
    TfPoseReaderOptions options);

  [[nodiscard]] const TfPoseReaderOptions & options() const noexcept;

  [[nodiscard]] TfPoseSnapshot read() const;

  [[nodiscard]] static TfPoseSnapshot evaluate_transform(
    const geometry_msgs::msg::TransformStamped & transform,
    const rclcpp::Time & now,
    double stale_timeout_sec);

  [[nodiscard]] static double normalize_yaw(
    double yaw_rad) noexcept;

private:
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer::SharedPtr buffer_;
  TfPoseReaderOptions options_;
};

}  // namespace savo_mapping

#endif  // SAVO_MAPPING__TF_POSE_READER_HPP_
