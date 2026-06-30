#pragma once

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

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

  static rclcpp::QoS camera_qos();
  static rclcpp::QoS odometry_qos();
  static rclcpp::QoS status_qos();

  void declare_parameters();
  void load_parameters();
  void create_publishers();
  void create_subscribers();

  void on_color_image(const Image::SharedPtr msg);
  void on_depth_image(const Image::SharedPtr msg);
  void on_camera_info(const CameraInfo::SharedPtr msg);

  void try_process_frame();

  bool has_required_inputs() const;
  CameraIntrinsics camera_intrinsics_from_info(const CameraInfo & info) const;

  void publish_odometry(const Odometry & odometry);
  void publish_status(const std::string & message);
  void publish_health(const std::string & message);
  void publish_tracking_quality(double score);

  std::string color_image_topic_;
  std::string color_camera_info_topic_;
  std::string depth_image_topic_;
  std::string depth_camera_info_topic_;

  std::string odom_topic_;
  std::string status_topic_;
  std::string health_topic_;
  std::string tracking_quality_topic_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string camera_frame_;

  double max_image_delay_s_{0.15};
  double max_depth_delay_s_{0.15};

  int min_features_{80};
  int good_features_target_{300};
  int max_features_{800};

  double min_tracking_quality_{0.35};
  double max_translation_jump_m_{0.30};
  double max_rotation_jump_rad_{0.35};

  bool publish_tf_{false};
  bool publish_diagnostics_{true};

  Image::SharedPtr latest_color_;
  Image::SharedPtr latest_depth_;
  CameraInfo::SharedPtr latest_camera_info_;

  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<String>::SharedPtr status_pub_;
  rclcpp::Publisher<String>::SharedPtr health_pub_;
  rclcpp::Publisher<Float32>::SharedPtr tracking_quality_pub_;

  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
};

}  // namespace savo_vo
