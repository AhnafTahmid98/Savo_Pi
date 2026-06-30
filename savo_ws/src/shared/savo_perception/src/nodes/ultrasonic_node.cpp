#include "savo_perception/ultrasonic_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace savo_perception
{
namespace
{

double safe_rate_hz(const double value, const double fallback)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }

  return value;
}

float distance_to_msg_value(
  const std::optional<double> & distance_m,
  const bool publish_nan_on_error)
{
  if (distance_m.has_value() && std::isfinite(*distance_m)) {
    return static_cast<float>(*distance_m);
  }

  if (publish_nan_on_error) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  return 0.0F;
}

}  // namespace

UltrasonicNode::UltrasonicNode(const rclcpp::NodeOptions & options)
: rclcpp::Node(constants::kUltrasonicNodeName, options)
{
  declare_parameters();
  load_parameters();
  setup_interfaces();
  start_reader();
}

void UltrasonicNode::declare_parameters()
{
  declare_parameter<int>("trig_pin", constants::kUltrasonicTrigPinDefault);
  declare_parameter<int>("echo_pin", constants::kUltrasonicEchoPinDefault);

  declare_parameter<double>("max_distance_m", constants::kUltrasonicMaxDistanceMDefault);
  declare_parameter<double>("valid_min_m", constants::kUltrasonicValidMinMDefault);
  declare_parameter<double>("valid_max_m", constants::kUltrasonicValidMaxMDefault);

  declare_parameter<double>("rate_hz", constants::kUltrasonicRateHzDefault);
  declare_parameter<int>("queue_len", 1);
  declare_parameter<std::string>("pin_factory", "lgpio");

  declare_parameter<int>("trigger_pulse_us", 10);
  declare_parameter<int>("echo_timeout_us", 30000);

  declare_parameter<std::string>("output_topic", topics::kUltrasonicFrontM);

  declare_parameter<bool>("publish_nan_on_error", constants::kPublishNanOnErrorDefault);
  declare_parameter<bool>("startup_fail_is_fatal", constants::kStartupFailIsFatalDefault);
}

void UltrasonicNode::load_parameters()
{
  config_.trig_pin = static_cast<int>(get_parameter("trig_pin").as_int());
  config_.echo_pin = static_cast<int>(get_parameter("echo_pin").as_int());

  config_.max_distance_m = get_parameter("max_distance_m").as_double();
  config_.valid_min_m = get_parameter("valid_min_m").as_double();
  config_.valid_max_m = get_parameter("valid_max_m").as_double();

  if (config_.valid_max_m < config_.valid_min_m) {
    std::swap(config_.valid_min_m, config_.valid_max_m);
  }

  config_.rate_hz = safe_rate_hz(
    get_parameter("rate_hz").as_double(),
    constants::kUltrasonicRateHzDefault);

  config_.queue_len = std::max(1, static_cast<int>(get_parameter("queue_len").as_int()));
  config_.pin_factory = get_parameter("pin_factory").as_string();

  config_.trigger_pulse_us = std::max(
    1,
    static_cast<int>(get_parameter("trigger_pulse_us").as_int()));

  config_.echo_timeout_us = std::max(
    1000,
    static_cast<int>(get_parameter("echo_timeout_us").as_int()));

  config_.output_topic = get_parameter("output_topic").as_string();

  config_.publish_nan_on_error = get_parameter("publish_nan_on_error").as_bool();
  config_.startup_fail_is_fatal = get_parameter("startup_fail_is_fatal").as_bool();

  if (!valid_ultrasonic_pins(config_.trig_pin, config_.echo_pin)) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid ultrasonic GPIO pins trig=%d echo=%d. Using defaults trig=%d echo=%d.",
      config_.trig_pin,
      config_.echo_pin,
      constants::kUltrasonicTrigPinDefault,
      constants::kUltrasonicEchoPinDefault);

    config_.trig_pin = constants::kUltrasonicTrigPinDefault;
    config_.echo_pin = constants::kUltrasonicEchoPinDefault;
  }
}

void UltrasonicNode::setup_interfaces()
{
  publisher_ = create_publisher<std_msgs::msg::Float32>(
    config_.output_topic,
    rclcpp::SensorDataQoS());

  const auto period = std::chrono::duration<double>(1.0 / config_.rate_hz);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {
      on_timer();
    });
}

void UltrasonicNode::start_reader()
{
  reader_ = std::make_unique<UltrasonicReader>(make_reader_config());

  if (reader_->start()) {
    reader_error_.clear();

    RCLCPP_INFO(
      get_logger(),
      "Ultrasonic node started: trig=%d echo=%d topic=%s rate=%.2fHz",
      config_.trig_pin,
      config_.echo_pin,
      config_.output_topic.c_str(),
      config_.rate_hz);

    return;
  }

  reader_error_ = reader_->last_error();

  RCLCPP_ERROR(
    get_logger(),
    "Ultrasonic reader failed to start: %s",
    reader_error_.c_str());

  if (config_.startup_fail_is_fatal) {
    throw std::runtime_error("Ultrasonic reader startup failed: " + reader_error_);
  }
}

void UltrasonicNode::stop_reader()
{
  if (reader_) {
    reader_->stop();
  }
}

void UltrasonicNode::on_timer()
{
  if (!reader_ || !reader_->started()) {
    publish_distance(std::nullopt);
    return;
  }

  const auto reading = reader_->read_once();
  publish_reading(reading);

  if (!reading.valid) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Ultrasonic read warning: %s",
      reading.error.c_str());
  }
}

void UltrasonicNode::publish_reading(const UltrasonicReading & reading)
{
  publish_distance(reading.filtered_m);
}

void UltrasonicNode::publish_distance(const std::optional<double> & distance_m)
{
  if (!publisher_) {
    return;
  }

  std_msgs::msg::Float32 msg;
  msg.data = distance_to_msg_value(distance_m, config_.publish_nan_on_error);
  publisher_->publish(msg);
}

UltrasonicConfig UltrasonicNode::make_reader_config() const
{
  UltrasonicConfig cfg;

  cfg.trig_pin = config_.trig_pin;
  cfg.echo_pin = config_.echo_pin;

  cfg.max_distance_m = config_.max_distance_m;
  cfg.valid_min_m = config_.valid_min_m;
  cfg.valid_max_m = config_.valid_max_m;

  cfg.queue_len = config_.queue_len;
  cfg.pin_factory = config_.pin_factory;

  cfg.trigger_pulse_us = config_.trigger_pulse_us;
  cfg.echo_timeout_us = config_.echo_timeout_us;

  cfg.sensor_name = "ultrasonic_front";
  cfg.source = "ultrasonic";

  return cfg;
}

}  // namespace savo_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<savo_perception::UltrasonicNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}