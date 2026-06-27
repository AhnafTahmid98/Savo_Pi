#ifndef SAVO_PERCEPTION__CMD_VEL_SAFETY_GATE_HPP_
#define SAVO_PERCEPTION__CMD_VEL_SAFETY_GATE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_perception/constants.hpp"
#include "savo_perception/safety_gate.hpp"
#include "savo_perception/topic_names.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

class SAVO_PERCEPTION_PUBLIC CmdVelSafetyGateNode : public rclcpp::Node
{
public:
  explicit CmdVelSafetyGateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_parameters();
  void setup_interfaces();

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_safety_stop(const std_msgs::msg::Bool::SharedPtr msg);
  void on_slowdown(const std_msgs::msg::Float32::SharedPtr msg);
  void on_safety_state(const std_msgs::msg::String::SharedPtr msg);
  void on_recovery_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);

  void on_timer();

  [[nodiscard]] SafetyGateInput make_gate_input() const;
  [[nodiscard]] SafetyGateOutput evaluate_gate() const;

  [[nodiscard]] VelocityCommand twist_to_command(
    const geometry_msgs::msg::Twist & msg,
    const std::string & source) const;

  [[nodiscard]] geometry_msgs::msg::Twist command_to_twist(
    const VelocityCommand & command) const;

  void publish_output(const SafetyGateOutput & output);
  void publish_gate_state(const SafetyGateOutput & output);

  [[nodiscard]] bool stamp_is_stale(
    const std::optional<std::chrono::steady_clock::time_point> & stamp,
    double timeout_s) const;

  SafetyGateConfig gate_config_{};

  std::string cmd_vel_topic_{topics::kCmdVel};
  std::string cmd_vel_safe_topic_{topics::kCmdVelSafe};

  std::string safety_stop_topic_{topics::kSafetyStop};
  std::string slowdown_topic_{topics::kSafetySlowdownFactor};
  std::string safety_state_topic_{topics::kSafetyState};

  std::string recovery_command_topic_{topics::kRecoveryCmdVel};
  std::string recovery_cmd_vel_safe_topic_{topics::kCmdVelSafe};

  std::string gate_state_topic_{topics::kCmdVelGateState};

  double loop_hz_{constants::kCmdVelGateLoopHzDefault};
  bool publish_gate_state_{true};

  VelocityCommand latest_command_{};
  bool has_command_{false};

  VelocityCommand latest_recovery_command_{};
  bool has_recovery_command_{false};

  bool latest_safety_stop_{true};
  std::optional<std::chrono::steady_clock::time_point> safety_stop_stamp_;

  double latest_slowdown_factor_{0.0};
  std::optional<std::chrono::steady_clock::time_point> slowdown_stamp_;

  std::string latest_safety_state_json_;
  std::optional<std::chrono::steady_clock::time_point> safety_state_stamp_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr recovery_cmd_vel_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr slowdown_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_safe_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gate_state_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__CMD_VEL_SAFETY_GATE_HPP_