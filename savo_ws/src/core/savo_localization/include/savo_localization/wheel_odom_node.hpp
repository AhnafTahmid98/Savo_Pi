#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "savo_localization/encoder_reader.hpp"
#include "savo_localization/encoder_state.hpp"
#include "savo_localization/mecanum_odom.hpp"

namespace savo_localization
{

class WheelOdomNode final : public rclcpp::Node
{
public:
  explicit WheelOdomNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~WheelOdomNode() override;

  WheelOdomNode(const WheelOdomNode &) = delete;
  WheelOdomNode & operator=(const WheelOdomNode &) = delete;

private:
  void declare_parameters();
  void load_parameters();
  void configure_encoder_reader();
  void configure_odometry();

  void timer_callback();

  void publish_odometry(
    const WheelOdomSample & odom_sample,
    const EncoderSample & encoder_sample);

  void publish_state(
    const WheelOdomSample & odom_sample,
    const EncoderSample & encoder_sample);

  void publish_transform(const WheelOdomSample & odom_sample);

  nav_msgs::msg::Odometry make_odometry_msg(
    const WheelOdomSample & odom_sample) const;

  geometry_msgs::msg::TransformStamped make_transform_msg(
    const WheelOdomSample & odom_sample) const;

  std::string make_state_json(
    const WheelOdomSample & odom_sample,
    const EncoderSample & encoder_sample) const;

  EncoderHardwareConfig make_encoder_config_from_params() const;
  MecanumGeometry make_mecanum_geometry_from_params() const;

  std::array<double, 36> pose_covariance() const;
  std::array<double, 36> twist_covariance() const;

  static std::array<double, 36> covariance_from_diagonal(
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw,
    double scale);

  static double yaw_to_quaternion_z(double yaw_rad);
  static double yaw_to_quaternion_w(double yaw_rad);

  static int counts_per_wheel_rev(
    int cpr,
    int decoding,
    double gear_ratio);

  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};

  std::string wheel_odom_topic_{"/wheel/odom"};
  std::string wheel_odom_state_topic_{"/savo_localization/wheel_odom_state"};

  double publish_rate_hz_{30.0};
  double timeout_s_{0.5};
  bool publish_tf_{false};

  double wheel_diameter_m_{0.065};
  double wheelbase_m_{0.165};
  double track_m_{0.165};

  int cpr_{20};
  int decoding_{4};
  double gear_ratio_{1.0};

  int gpiochip_{-1};
  double poll_s_{0.001};
  double debounce_s_{0.0003};
  bool use_internal_pullup_{false};
  bool use_hw_debounce_{true};

  int fl_a_gpio_{21};
  int fl_b_gpio_{20};
  int fr_a_gpio_{13};
  int fr_b_gpio_{25};
  int rl_a_gpio_{23};
  int rl_b_gpio_{24};
  int rr_a_gpio_{12};
  int rr_b_gpio_{26};

  bool invert_fl_{false};
  bool invert_fr_{false};
  bool invert_rl_{false};
  bool invert_rr_{false};

  bool reset_pose_on_start_{true};
  double start_x_m_{0.0};
  double start_y_m_{0.0};
  double start_yaw_rad_{0.0};

  double odom_covariance_scale_{1.0};

  double pose_x_covariance_{0.05};
  double pose_y_covariance_{0.10};
  double pose_z_covariance_{999.0};
  double pose_roll_covariance_{999.0};
  double pose_pitch_covariance_{999.0};
  double pose_yaw_covariance_{0.10};

  double twist_vx_covariance_{0.05};
  double twist_vy_covariance_{0.10};
  double twist_vz_covariance_{999.0};
  double twist_wx_covariance_{999.0};
  double twist_wy_covariance_{999.0};
  double twist_wz_covariance_{0.10};

  std::unique_ptr<EncoderReader> encoder_reader_;
  std::unique_ptr<MecanumOdom> odom_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_update_time_;
  bool have_last_update_{false};

  std::uint64_t loop_count_{0};
  std::uint64_t publish_count_{0};
  std::uint64_t error_count_{0};
};

}  // namespace savo_localization
