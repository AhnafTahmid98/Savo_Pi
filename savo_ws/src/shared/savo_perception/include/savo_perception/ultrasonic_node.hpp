#ifndef SAVO_PERCEPTION__ULTRASONIC_NODE_HPP_
#define SAVO_PERCEPTION__ULTRASONIC_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "savo_perception/constants.hpp"
#include "savo_perception/topic_names.hpp"
#include "savo_perception/ultrasonic_reader.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

struct SAVO_PERCEPTION_PUBLIC UltrasonicNodeConfig
{
  int trig_pin{constants::kUltrasonicTrigPinDefault};
  int echo_pin{constants::kUltrasonicEchoPinDefault};

  double max_distance_m{constants::kUltrasonicMaxDistanceMDefault};
  double valid_min_m{constants::kUltrasonicValidMinMDefault};
  double valid_max_m{constants::kUltrasonicValidMaxMDefault};

  double rate_hz{constants::kUltrasonicRateHzDefault};
  int queue_len{1};
  std::string pin_factory{"lgpio"};

  int gpiochip{4};
  int trigger_pulse_us{10};
  int echo_timeout_us{30000};
  int echo_idle_timeout_us{30000};

  std::string output_topic{topics::kUltrasonicFrontM};

  bool publish_nan_on_error{constants::kPublishNanOnErrorDefault};
  bool startup_fail_is_fatal{constants::kStartupFailIsFatalDefault};
};

class SAVO_PERCEPTION_PUBLIC UltrasonicNode : public rclcpp::Node
{
public:
  explicit UltrasonicNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_parameters();
  void setup_interfaces();

  void start_reader();
  void stop_reader();

  void on_timer();

  void publish_reading(const UltrasonicReading & reading);
  void publish_distance(const std::optional<double> & distance_m);

  [[nodiscard]] UltrasonicConfig make_reader_config() const;

  UltrasonicNodeConfig config_{};

  std::unique_ptr<UltrasonicReader> reader_;
  std::string reader_error_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__ULTRASONIC_NODE_HPP_