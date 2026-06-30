#ifndef SAVO_PERCEPTION__RANGE_HEALTH_NODE_HPP_
#define SAVO_PERCEPTION__RANGE_HEALTH_NODE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_perception/constants.hpp"
#include "savo_perception/perception_types.hpp"
#include "savo_perception/topic_names.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

class SAVO_PERCEPTION_PUBLIC RangeHealthNode : public rclcpp::Node
{
public:
  explicit RangeHealthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_parameters();
  void setup_interfaces();

  void on_depth_front(const std_msgs::msg::Float32::SharedPtr msg);
  void on_tof_left(const std_msgs::msg::Float32::SharedPtr msg);
  void on_tof_right(const std_msgs::msg::Float32::SharedPtr msg);
  void on_ultrasonic_front(const std_msgs::msg::Float32::SharedPtr msg);

  void on_timer();
  void on_heartbeat_timer();

  [[nodiscard]] RangeSample sample_from_value(
    const std::string & sensor_name,
    float value,
    const std::string & source) const;

  [[nodiscard]] RangeSample missing_sample(
    const std::string & sensor_name,
    const std::string & reason,
    bool stale_now) const;

  [[nodiscard]] std::vector<SensorHealth> current_health() const;
  [[nodiscard]] bool overall_ok(const std::vector<SensorHealth> & health) const;
  [[nodiscard]] std::string overall_status(const std::vector<SensorHealth> & health) const;

  [[nodiscard]] bool is_required_sensor(const std::string & sensor_name) const;
  [[nodiscard]] bool is_optional_sensor(const std::string & sensor_name) const;

  [[nodiscard]] std::vector<std::string> stale_required_sensors(
    const std::vector<SensorHealth> & health) const;

  [[nodiscard]] std::vector<std::string> error_required_sensors(
    const std::vector<SensorHealth> & health) const;

  void publish_health(const std::vector<SensorHealth> & health);
  void publish_sensor_status(const std::vector<SensorHealth> & health);
  void publish_heartbeat();

  [[nodiscard]] std::string health_to_json(const std::vector<SensorHealth> & health) const;
  [[nodiscard]] std::string sensor_health_to_json(const SensorHealth & health) const;
  [[nodiscard]] std::string string_list_to_json(const std::vector<std::string> & values) const;

  std::string depth_front_topic_{topics::kDepthFrontM};
  std::string tof_left_topic_{topics::kTofLeftM};
  std::string tof_right_topic_{topics::kTofRightM};
  std::string ultrasonic_front_topic_{topics::kUltrasonicFrontM};

  std::string range_health_topic_{topics::kRangeHealth};
  std::string sensor_status_topic_{topics::kSensorStatus};
  std::string heartbeat_topic_{topics::kHeartbeat};

  double publish_hz_{constants::kRangeHealthPublishHzDefault};
  double stale_timeout_s_{constants::kSensorStaleTimeoutSDefault};
  double heartbeat_hz_{1.0};

  bool include_depth_in_overall_ok_{false};
  bool depth_front_required_{false};

  bool publish_json_{true};
  bool publish_compact_status_{true};

  std::vector<std::string> required_sensors_{
    "tof_left",
    "tof_right",
    "ultrasonic_front"};

  std::vector<std::string> optional_sensors_{
    "depth_front"};

  RangeSample depth_front_;
  RangeSample tof_left_;
  RangeSample tof_right_;
  RangeSample ultrasonic_front_;

  std::uint64_t heartbeat_count_{0};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_front_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tof_right_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ultrasonic_front_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr range_health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__RANGE_HEALTH_NODE_HPP_