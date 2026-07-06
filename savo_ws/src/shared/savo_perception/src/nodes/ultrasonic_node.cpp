#include "savo_perception/ultrasonic_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace savo_perception
{
namespace
{

double positive_or_default(const double value, const double fallback)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }

  return value;
}

int positive_int_or_default(const int value, const int fallback)
{
  if (value <= 0) {
    return fallback;
  }

  return value;
}

bool clear_no_echo_error(const std::string & error)
{
  return error == "no_echo" || error == "echo_start_timeout";
}

}  // namespace

UltrasonicNode::UltrasonicNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ultrasonic_node", options)
{
  declare_parameters();
  load_parameters();
  setup_interfaces();
  start_reader();
}

void UltrasonicNode::declare_parameters()
{
  this->declare_parameter<int>("trig_pin", config_.trig_pin);
  this->declare_parameter<int>("echo_pin", config_.echo_pin);

  this->declare_parameter<double>("max_distance_m", config_.max_distance_m);
  this->declare_parameter<double>("valid_min_m", config_.valid_min_m);
  this->declare_parameter<double>("valid_max_m", config_.valid_max_m);

  this->declare_parameter<double>("rate_hz", config_.rate_hz);
  this->declare_parameter<int>("queue_len", config_.queue_len);
  this->declare_parameter<std::string>("pin_factory", config_.pin_factory);

  this->declare_parameter<int>("gpiochip", config_.gpiochip);
  this->declare_parameter<int>("trigger_pulse_us", config_.trigger_pulse_us);
  this->declare_parameter<int>("echo_timeout_us", config_.echo_timeout_us);
  this->declare_parameter<int>("echo_idle_timeout_us", config_.echo_idle_timeout_us);

  this->declare_parameter<std::string>("output_topic", config_.output_topic);

  this->declare_parameter<bool>("publish_nan_on_error", config_.publish_nan_on_error);
  this->declare_parameter<bool>("startup_fail_is_fatal", config_.startup_fail_is_fatal);
}

void UltrasonicNode::load_parameters()
{
  config_.trig_pin = this->get_parameter("trig_pin").as_int();
  config_.echo_pin = this->get_parameter("echo_pin").as_int();

  config_.max_distance_m = positive_or_default(
    this->get_parameter("max_distance_m").as_double(),
    constants::kUltrasonicMaxDistanceMDefault);

  config_.valid_min_m = std::max(0.0, this->get_parameter("valid_min_m").as_double());
  config_.valid_max_m = positive_or_default(
    this->get_parameter("valid_max_m").as_double(),
    config_.max_distance_m);

  if (config_.valid_max_m < config_.valid_min_m) {
    std::swap(config_.valid_min_m, config_.valid_max_m);
  }

  config_.rate_hz = positive_or_default(
    this->get_parameter("rate_hz").as_double(),
    constants::kUltrasonicRateHzDefault);

  config_.queue_len = positive_int_or_default(
    this->get_parameter("queue_len").as_int(),
    1);

  config_.pin_factory = this->get_parameter("pin_factory").as_string();

  config_.gpiochip = static_cast<int>(this->get_parameter("gpiochip").as_int());
  config_.trigger_pulse_us = positive_int_or_default(
    this->get_parameter("trigger_pulse_us").as_int(),
    10);
  config_.echo_timeout_us = positive_int_or_default(
    this->get_parameter("echo_timeout_us").as_int(),
    30000);
  config_.echo_idle_timeout_us = positive_int_or_default(
    this->get_parameter("echo_idle_timeout_us").as_int(),
    30000);

  config_.output_topic = this->get_parameter("output_topic").as_string();

  config_.publish_nan_on_error = this->get_parameter("publish_nan_on_error").as_bool();
  config_.startup_fail_is_fatal = this->get_parameter("startup_fail_is_fatal").as_bool();
}

void UltrasonicNode::setup_interfaces()
{
  publisher_ = this->create_publisher<std_msgs::msg::Float32>(
    config_.output_topic,
    rclcpp::SensorDataQoS());

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / config_.rate_hz));

  timer_ = this->create_wall_timer(
    period,
    [this]() {
      this->on_timer();
    });
}

void UltrasonicNode::start_reader()
{
  reader_ = std::make_unique<UltrasonicReader>(make_reader_config());

  if (!reader_->start()) {
    reader_error_ = reader_->last_error();

    const std::string message = "Ultrasonic reader failed to start: " + reader_error_;
    if (config_.startup_fail_is_fatal) {
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
      throw std::runtime_error(message);
    }

    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Ultrasonic node started: trig=%d echo=%d gpiochip=%d topic=%s rate=%.2fHz "
    "valid=[%.3f, %.3f]m timeout=%dus idle_timeout=%dus",
    config_.trig_pin,
    config_.echo_pin,
    reader_->gpiochip_number(),
    config_.output_topic.c_str(),
    config_.rate_hz,
    config_.valid_min_m,
    config_.valid_max_m,
    config_.echo_timeout_us,
    config_.echo_idle_timeout_us);
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
    if (config_.publish_nan_on_error) {
      publish_distance(std::nullopt);
    }
    return;
  }

  const auto reading = reader_->read_once();
  publish_reading(reading);
}

void UltrasonicNode::publish_reading(const UltrasonicReading & reading)
{
  if (!reading.valid) {
    if (clear_no_echo_error(reading.error)) {
      publish_distance(std::numeric_limits<double>::infinity());
      return;
    }

    if (!reading.error.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Ultrasonic read warning: %s",
        reading.error.c_str());
    }

    if (config_.publish_nan_on_error) {
      publish_distance(std::nullopt);
    }
    return;
  }

  publish_distance(reading.filtered_m);
}

void UltrasonicNode::publish_distance(const std::optional<double> & distance_m)
{
  std_msgs::msg::Float32 msg;

  if (distance_m.has_value()) {
    const double value = *distance_m;
    if (std::isfinite(value) || value == std::numeric_limits<double>::infinity()) {
      msg.data = static_cast<float>(value);
    } else {
      msg.data = std::numeric_limits<float>::quiet_NaN();
    }
  } else {
    msg.data = std::numeric_limits<float>::quiet_NaN();
  }

  publisher_->publish(msg);
}

UltrasonicConfig UltrasonicNode::make_reader_config() const
{
  UltrasonicConfig reader_config;

  reader_config.trig_pin = config_.trig_pin;
  reader_config.echo_pin = config_.echo_pin;

  reader_config.max_distance_m = config_.max_distance_m;
  reader_config.valid_min_m = config_.valid_min_m;
  reader_config.valid_max_m = config_.valid_max_m;

  reader_config.queue_len = config_.queue_len;
  reader_config.pin_factory = config_.pin_factory;

  reader_config.gpiochip = config_.gpiochip;
  reader_config.trigger_pulse_us = config_.trigger_pulse_us;
  reader_config.echo_timeout_us = config_.echo_timeout_us;
  reader_config.echo_idle_timeout_us = config_.echo_idle_timeout_us;

  reader_config.sensor_name = "ultrasonic_front";
  reader_config.source = "ultrasonic";

  return reader_config;
}

}  // namespace savo_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<savo_perception::UltrasonicNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(rclcpp::get_logger("ultrasonic_node"), "%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
