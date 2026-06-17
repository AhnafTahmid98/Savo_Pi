#pragma once

#include "savo_base/base_safety.hpp"
#include "savo_base/base_state.hpp"
#include "savo_base/base_types.hpp"
#include "savo_base/command_guard.hpp"
#include "savo_base/freenove_motor_board.hpp"
#include "savo_base/mecanum_mixer.hpp"
#include "savo_base/timeout_watchdog.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

namespace savo_base
{

class BaseDriverNode final : public rclcpp::Node
{
public:
  explicit BaseDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BaseDriverNode() override;

  BaseDriverNode(const BaseDriverNode &) = delete;
  BaseDriverNode & operator=(const BaseDriverNode &) = delete;

private:
  void declare_parameters();
  void load_parameters();
  void configure_runtime();

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void safety_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void slowdown_callback(const std_msgs::msg::Float32::SharedPtr msg);

  void loop_callback();
  void publish_state(
    const SafetyDecision & decision,
    const WatchdogStatus & watchdog,
    const WheelDuty & duty);

  double command_age_s() const;

  static double clamp_double(double value, double min_value, double max_value);
  static int clamp_int(int value, int min_value, int max_value);
  static std::string bool_text(bool value);

  BaseDriverConfig config_{};

  std::unique_ptr<CommandGuard> command_guard_;
  std::unique_ptr<TimeoutWatchdog> watchdog_;
  std::unique_ptr<BaseSafety> safety_;
  std::unique_ptr<MecanumMixer> mixer_;
  std::unique_ptr<FreenoveMotorBoard> board_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr slowdown_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;

  geometry_msgs::msg::Twist latest_cmd_{};
  rclcpp::Time latest_cmd_time_{0, 0, RCL_ROS_TIME};

  bool has_command_{false};
  bool safety_stop_{false};
  double slowdown_factor_{1.0};

  BaseCounters counters_{};
  std::string last_board_error_;
};

}  // namespace savo_base
