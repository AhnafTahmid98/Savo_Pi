#ifndef SAVO_PERCEPTION__SAFETY_STOP_NODE_HPP_
#define SAVO_PERCEPTION__SAFETY_STOP_NODE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_perception/constants.hpp"
#include "savo_perception/perception_types.hpp"
#include "savo_perception/safety_policy.hpp"
#include "savo_perception/topic_names.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

class SAVO_PERCEPTION_PUBLIC SafetyStopNode : public rclcpp::Node
{
public:
  explicit SafetyStopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_parameters();
  void setup_interfaces();

  void on_depth_front(const std_msgs::msg::Float32::SharedPtr msg);
  void on_tof_left(const std_msgs::msg::Float32::SharedPtr msg);
  void on_tof_right(const std_msgs::msg::Float32::SharedPtr msg);
  void on_ultrasonic_front(const std_msgs::msg::Float32::SharedPtr msg);

  void on_timer();

  [[nodiscard]] RangeSample sample_from_value(
    const std::string & sensor_name,
    float value,
    const std::string & source) const;

  [[nodiscard]] RangeSample missing_sample(
    const std::string & sensor_name,
    const std::string & reason,
    bool stale_now) const;

  [[nodiscard]] RangeSnapshot current_snapshot() const;

  void publish_decision(const SafetyPolicyUpdate & update);

  [[nodiscard]] std::string decision_to_json(const SafetyDecision & decision) const;
  [[nodiscard]] std::string state_to_json(const SafetyState & state) const;
  [[nodiscard]] std::string string_list_to_json(const std::vector<std::string> & values) const;

  SafetyPolicyConfig policy_config_{};
  SafetyPolicy policy_{};

  std::string depth_front_topic_{topics::kDepthFrontM};
  std::string tof_left_topic_{topics::kTofLeftM};
  std::string tof_right_topic_{topics::kTofRightM};
  std::string ultrasonic_front_topic_{topics::kUltrasonicFrontM};

  std::string safety_stop_topic_{topics::kSafetyStop};
  std::string slowdown_topic_{topics::kSafetySlowdownFactor};
  std::string safety_state_topic_{topics::kSafetyState};

  double loop_hz_{constants::kSafetyLoopHzDefault};
  bool publish_state_json_{true};

  RangeSample depth_front_;
  RangeSample tof_left_;
  RangeSample tof_right_;
  RangeSample ultrasonic_front_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_front_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_right_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ultrasonic_front_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slowdown_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_state_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__SAFETY_STOP_NODE_HPP_