#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "savo_localization/encoder_reader.hpp"
#include "savo_localization/encoder_state.hpp"
#include "savo_localization/mecanum_odom.hpp"
#include "savo_localization/producer_health.hpp"
#include "savo_localization/wheel_joint_state.hpp"

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

  void publish_health_outputs(std::int64_t monotonic_time_ns, bool force);
  void publish_state(const ProducerHealthSnapshot & snapshot);
  void publish_debug_state(
    const WheelOdomSample & odom_sample,
    const EncoderSample & encoder_sample);

  void publish_joint_state(
    const EncoderSample & encoder_sample,
    const rclcpp::Time & measurement_time);

  void publish_transform(const WheelOdomSample & odom_sample);

  nav_msgs::msg::Odometry make_odometry_msg(
    const WheelOdomSample & odom_sample) const;

  geometry_msgs::msg::TransformStamped make_transform_msg(
    const WheelOdomSample & odom_sample) const;

  std::string make_state_json(
    const WheelOdomSample & odom_sample,
    const EncoderSample & encoder_sample) const;
  [[nodiscard]] ProducerHealthSnapshot make_health_snapshot(
    std::int64_t monotonic_time_ns) const;
  [[nodiscard]] bool sample_data_valid(
    const WheelOdomSample & odom_sample,
    const EncoderSample & encoder_sample) const;
  [[nodiscard]] bool output_due(
    std::int64_t monotonic_time_ns,
    std::int64_t last_publish_time_ns,
    double rate_hz) const;

  EncoderHardwareConfig make_encoder_config_from_params() const;
  MecanumGeometry make_mecanum_geometry_from_params() const;
  WheelSpeeds filter_wheel_speeds(const WheelSpeeds & raw_wheel_speeds);
  static double apply_deadband(double value, double deadband);
  static int active_wheel_count_from_speeds(
    const WheelSpeeds & wheel_speeds,
    double deadband);

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
  std::string base_frame_id_{"base_footprint"};

  std::string wheel_odom_topic_{"/wheel/odom"};
  std::string wheel_odom_state_topic_{"/savo_localization/wheel_odom_state"};
  std::string wheel_odom_debug_topic_{"/savo_localization/wheel_odom_debug"};
  std::string joint_states_topic_{"/joint_states"};

  double publish_rate_hz_{30.0};
  double health_publish_rate_hz_{5.0};
  double debug_publish_rate_hz_{2.0};
  double timestamp_fault_hold_s_{2.0};
  std::size_t producer_rate_window_size_{30U};
  double timeout_s_{0.5};
  bool publish_tf_{false};
  bool publish_joint_states_{true};
  bool publish_debug_state_{false};
  int max_encoder_illegal_transitions_{20};

  double wheel_diameter_m_{0.065};
  double wheelbase_m_{0.160};
  double track_m_{0.216};

  int cpr_{20};
  int decoding_{4};
  double gear_ratio_{1.0};

  int gpiochip_{-1};
  double poll_s_{0.001};
  double debounce_s_{0.0003};
  bool use_internal_pullup_{false};
  bool use_hw_debounce_{true};

  int fl_a_gpio_{20};
  int fl_b_gpio_{21};
  int fr_a_gpio_{13};
  int fr_b_gpio_{25};
  int rl_a_gpio_{24};
  int rl_b_gpio_{23};
  int rr_a_gpio_{12};
  int rr_b_gpio_{26};

  bool invert_fl_{false};
  bool invert_fr_{false};
  bool invert_rl_{false};
  bool invert_rr_{false};

  bool velocity_filter_enabled_{true};
  double velocity_ema_alpha_{0.25};
  double wheel_speed_deadband_mps_{0.02};
  int min_active_wheels_for_twist_{2};

  WheelSpeeds filtered_wheel_speeds_{};
  bool have_filtered_wheel_speeds_{false};

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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_update_time_;
  bool have_last_update_{false};

  std::uint64_t loop_count_{0};
  std::uint64_t publish_count_{0};
  std::uint64_t error_count_{0};
  bool update_error_active_{false};
  bool last_sample_data_valid_{false};
  std::optional<WheelOdomSample> last_odom_sample_{};
  std::optional<EncoderSample> last_encoder_sample_{};
  ProducerRateTracker producer_rate_tracker_{};
  std::int64_t timestamp_fault_until_ns_{0};
  std::int64_t last_health_publish_ns_{-1};
  std::int64_t last_debug_publish_ns_{-1};
  std::string last_health_state_{};
  std::string last_health_reason_{};
};

}  // namespace savo_localization
