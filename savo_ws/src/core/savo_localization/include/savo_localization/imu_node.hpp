#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_localization/bno055_driver.hpp"

namespace savo_localization
{

class ImuNode final : public rclcpp::Node
{
public:
  explicit ImuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImuNode() override;

  ImuNode(const ImuNode &) = delete;
  ImuNode & operator=(const ImuNode &) = delete;

private:
  void declare_parameters();
  void load_parameters();
  void configure_driver();

  void timer_callback();

  void publish_imu(const BNO055Sample & sample);
  void publish_state(const BNO055Sample & sample);
  void publish_diagnostics(const BNO055Sample & sample);

  sensor_msgs::msg::Imu make_imu_msg(const BNO055Sample & sample) const;
  std_msgs::msg::String make_state_msg(const BNO055Sample & sample) const;
  diagnostic_msgs::msg::DiagnosticArray make_diagnostic_msg(
    const BNO055Sample & sample) const;

  std::string make_state_json(const BNO055Sample & sample) const;

  BNO055Mode configured_mode() const;

  std::array<double, 9> orientation_covariance() const;
  std::array<double, 9> angular_velocity_covariance() const;
  std::array<double, 9> linear_acceleration_covariance() const;

  static std::array<double, 9> covariance3_from_diagonal(
    double x,
    double y,
    double z);

  static double deg_to_rad(double value_deg);
  static double yaw_to_quaternion_z(double yaw_rad);
  static double yaw_to_quaternion_w(double yaw_rad);

  static int diagnostic_level_from_sample(const BNO055Sample & sample);
  static std::string diagnostic_message_from_sample(const BNO055Sample & sample);

  std::string frame_id_{"imu_link"};
  std::string imu_topic_{"/imu/data"};
  std::string imu_state_topic_{"/savo_localization/imu_state"};
  std::string diagnostics_topic_{"/diagnostics"};

  int i2c_bus_{1};
  int i2c_address_{0x28};
  std::string mode_{"ndof"};

  double publish_rate_hz_{25.0};
  bool reset_on_start_{true};

  bool publish_orientation_{true};
  bool publish_magnetic_field_{true};
  bool publish_temperature_{true};
  bool publish_diagnostics_{true};

  double orientation_covariance_roll_{0.05};
  double orientation_covariance_pitch_{0.05};
  double orientation_covariance_yaw_{0.10};

  double angular_velocity_covariance_x_{0.02};
  double angular_velocity_covariance_y_{0.02};
  double angular_velocity_covariance_z_{0.02};

  double linear_acceleration_covariance_x_{0.10};
  double linear_acceleration_covariance_y_{0.10};
  double linear_acceleration_covariance_z_{0.10};

  std::unique_ptr<BNO055Driver> driver_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::uint64_t sample_count_{0};
  std::uint64_t publish_count_{0};
  std::uint64_t error_count_{0};

  rclcpp::Time last_sample_time_;
  bool have_last_sample_{false};
};

}  // namespace savo_localization