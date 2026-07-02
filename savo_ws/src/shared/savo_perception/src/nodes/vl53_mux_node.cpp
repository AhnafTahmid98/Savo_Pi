#include "savo_perception/vl53_mux_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
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

int safe_median_window(const int value)
{
  if (valid_vl53_median_window(value)) {
    return value;
  }

  return constants::kVl53MedianWindowDefault;
}

std::uint8_t as_u8(const int value)
{
  return static_cast<std::uint8_t>(std::clamp(value, 0, 255));
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

std::string optional_distance_text(const std::optional<double> & value)
{
  if (!value.has_value()) {
    return "none";
  }

  std::ostringstream stream;
  stream << *value;
  return stream.str();
}

std::string reading_text(const Vl53l1xReading & reading)
{
  std::ostringstream stream;
  stream
    << "valid=" << (reading.valid ? "true" : "false")
    << " raw=" << optional_distance_text(reading.raw_m)
    << " filtered=" << optional_distance_text(reading.filtered_m)
    << " error='" << reading.error << "'";
  return stream.str();
}

}  // namespace

Vl53MuxNode::Vl53MuxNode(const rclcpp::NodeOptions & options)
: rclcpp::Node(constants::kVl53MuxNodeName, options)
{
  declare_parameters();
  load_parameters();
  setup_interfaces();
  start_driver();
}

void Vl53MuxNode::declare_parameters()
{
  declare_parameter<int>("bus", constants::kI2cBusDefault);

  declare_parameter<int>("tca_addr", constants::kTca9548aAddrDefault);
  declare_parameter<int>("vl53_addr", constants::kVl53l1xAddrDefault);

  declare_parameter<int>("right_channel", constants::kVl53RightChannelDefault);
  declare_parameter<int>("left_channel", constants::kVl53LeftChannelDefault);

  declare_parameter<double>("rate_hz", constants::kVl53RateHzDefault);
  declare_parameter<int>("median_window", constants::kVl53MedianWindowDefault);

  declare_parameter<double>("settle_s", constants::kVl53SettleSDefault);
  declare_parameter<double>("init_settle_s", constants::kVl53InitSettleSDefault);

  declare_parameter<double>("valid_min_m", constants::kVl53ValidMinMDefault);
  declare_parameter<double>("valid_max_m", constants::kVl53ValidMaxMDefault);

  declare_parameter<std::string>("left_topic", topics::kTofLeftM);
  declare_parameter<std::string>("right_topic", topics::kTofRightM);

  declare_parameter<bool>("publish_nan_on_error", constants::kPublishNanOnErrorDefault);
  declare_parameter<bool>("startup_fail_is_fatal", constants::kStartupFailIsFatalDefault);
}

void Vl53MuxNode::load_parameters()
{
  config_.bus = static_cast<int>(get_parameter("bus").as_int());

  config_.tca_addr = as_u8(static_cast<int>(get_parameter("tca_addr").as_int()));
  config_.vl53_addr = as_u8(static_cast<int>(get_parameter("vl53_addr").as_int()));

  config_.right_channel = static_cast<int>(get_parameter("right_channel").as_int());
  config_.left_channel = static_cast<int>(get_parameter("left_channel").as_int());

  config_.rate_hz = safe_rate_hz(
    get_parameter("rate_hz").as_double(),
    constants::kVl53RateHzDefault);

  config_.median_window = safe_median_window(
    static_cast<int>(get_parameter("median_window").as_int()));

  config_.settle_s = std::max(0.0, get_parameter("settle_s").as_double());
  config_.init_settle_s = std::max(0.0, get_parameter("init_settle_s").as_double());

  config_.valid_min_m = get_parameter("valid_min_m").as_double();
  config_.valid_max_m = get_parameter("valid_max_m").as_double();

  if (config_.valid_max_m < config_.valid_min_m) {
    std::swap(config_.valid_min_m, config_.valid_max_m);
  }

  config_.left_topic = get_parameter("left_topic").as_string();
  config_.right_topic = get_parameter("right_topic").as_string();

  config_.publish_nan_on_error = get_parameter("publish_nan_on_error").as_bool();
  config_.startup_fail_is_fatal = get_parameter("startup_fail_is_fatal").as_bool();

  if (!valid_tca_channel(config_.right_channel)) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid right_channel=%d. Using default %d.",
      config_.right_channel,
      constants::kVl53RightChannelDefault);

    config_.right_channel = constants::kVl53RightChannelDefault;
  }

  if (!valid_tca_channel(config_.left_channel)) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid left_channel=%d. Using default %d.",
      config_.left_channel,
      constants::kVl53LeftChannelDefault);

    config_.left_channel = constants::kVl53LeftChannelDefault;
  }
}

void Vl53MuxNode::setup_interfaces()
{
  left_pub_ = create_publisher<std_msgs::msg::Float32>(
    config_.left_topic,
    rclcpp::SensorDataQoS());

  right_pub_ = create_publisher<std_msgs::msg::Float32>(
    config_.right_topic,
    rclcpp::SensorDataQoS());

  const auto period = std::chrono::duration<double>(1.0 / config_.rate_hz);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {
      on_timer();
    });
}

void Vl53MuxNode::start_driver()
{
  driver_ = std::make_unique<Vl53MuxPairDriver>(make_driver_config());

  if (driver_->start()) {
    driver_error_.clear();

    RCLCPP_INFO(
      get_logger(),
      "VL53 mux started: bus=%d tca=0x%02X vl53=0x%02X right_ch=%d left_ch=%d rate=%.2fHz",
      config_.bus,
      static_cast<int>(config_.tca_addr),
      static_cast<int>(config_.vl53_addr),
      config_.right_channel,
      config_.left_channel,
      config_.rate_hz);

    return;
  }

  driver_error_ = driver_->last_error();

  RCLCPP_ERROR(
    get_logger(),
    "VL53 mux driver failed to start: %s",
    driver_error_.c_str());

  if (config_.startup_fail_is_fatal) {
    throw std::runtime_error("VL53 mux driver startup failed: " + driver_error_);
  }
}

void Vl53MuxNode::stop_driver()
{
  if (driver_) {
    driver_->stop();
  }
}

void Vl53MuxNode::on_timer()
{
  publish_distance(left_pub_, std::nullopt);
  publish_distance(right_pub_, std::nullopt);

  if (!driver_ || !driver_->started()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "VL53 driver not started: %s",
      driver_error_.c_str());
    return;
  }

  Vl53MuxPairReading reading;

  try {
    reading = driver_->read_once();
  } catch (const std::exception & exc) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "VL53 read exception: %s",
      exc.what());
    return;
  }

  if (reading.left.valid) {
    publish_reading(left_pub_, reading.left);
  }

  if (reading.right.valid) {
    publish_reading(right_pub_, reading.right);
  }

  if (!reading.left.valid || !reading.right.valid) {
    const auto left_text = reading_text(reading.left);
    const auto right_text = reading_text(reading.right);

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "VL53 read warning: left={%s} right={%s}",
      left_text.c_str(),
      right_text.c_str());
  }
}

void Vl53MuxNode::publish_reading(
  const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher,
  const Vl53l1xReading & reading)
{
  publish_distance(publisher, reading.filtered_m);
}

void Vl53MuxNode::publish_distance(
  const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher,
  const std::optional<double> & distance_m)
{
  if (!publisher) {
    return;
  }

  std_msgs::msg::Float32 msg;
  msg.data = distance_to_msg_value(distance_m, config_.publish_nan_on_error);
  publisher->publish(msg);
}

Vl53MuxPairConfig Vl53MuxNode::make_driver_config() const
{
  Vl53MuxPairConfig cfg;

  cfg.bus = config_.bus;
  cfg.tca_address = config_.tca_addr;
  cfg.sensor_address = config_.vl53_addr;

  cfg.right_channel = config_.right_channel;
  cfg.left_channel = config_.left_channel;

  cfg.settle_s = config_.settle_s;
  cfg.init_settle_s = config_.init_settle_s;

  cfg.valid_min_m = config_.valid_min_m;
  cfg.valid_max_m = config_.valid_max_m;

  cfg.median_window = config_.median_window;

  return cfg;
}

}  // namespace savo_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const auto node = std::make_shared<savo_perception::Vl53MuxNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(rclcpp::get_logger("vl53_mux_node"), "%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}