#include "savo_perception/vl53_mux_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

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
  start_worker();
}

Vl53MuxNode::~Vl53MuxNode()
{
  request_worker_stop();

  if (worker_thread_.joinable()) {
    worker_thread_.detach();
  } else {
    stop_driver();
  }
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

  declare_parameter<double>("stale_timeout_s", config_.stale_timeout_s);
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

  config_.stale_timeout_s = safe_rate_hz(
    get_parameter("stale_timeout_s").as_double(),
    0.50);

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
  driver_ = std::make_shared<Vl53MuxPairDriver>(make_driver_config());

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

void Vl53MuxNode::start_worker()
{
  if (!driver_ || !driver_->started()) {
    RCLCPP_WARN(
      get_logger(),
      "VL53 worker not started because driver is not ready: %s",
      driver_error_.c_str());
    return;
  }

  worker_state_ = std::make_shared<Vl53WorkerState>();
  worker_state_->stop_requested.store(false);

  worker_thread_ = std::thread(
    &Vl53MuxNode::worker_loop,
    worker_state_,
    driver_,
    config_);

  RCLCPP_INFO(get_logger(), "VL53 worker thread started");
}

void Vl53MuxNode::request_worker_stop()
{
  if (worker_state_) {
    worker_state_->stop_requested.store(true);
  }
}

void Vl53MuxNode::worker_loop(
  std::shared_ptr<Vl53WorkerState> state,
  std::shared_ptr<Vl53MuxPairDriver> driver,
  Vl53MuxNodeConfig config)
{
  const auto period = std::chrono::duration<double>(
    1.0 / safe_rate_hz(config.rate_hz, constants::kVl53RateHzDefault));

  while (state && driver && !state->stop_requested.load()) {
    const auto loop_start = std::chrono::steady_clock::now();

    Vl53MuxPairReading reading;
    std::string driver_error;

    try {
      reading = driver->read_once();
      driver_error = driver->last_error();
    } catch (const std::exception & exc) {
      driver_error = std::string("read_exception:") + exc.what();
      reading.left.sensor_name = "tof_left";
      reading.left.error = driver_error;
      reading.right.sensor_name = "tof_right";
      reading.right.error = driver_error;
    } catch (...) {
      driver_error = "read_exception:unknown";
      reading.left.sensor_name = "tof_left";
      reading.left.error = driver_error;
      reading.right.sensor_name = "tof_right";
      reading.right.error = driver_error;
    }

    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->latest.left = reading.left;
      state->latest.right = reading.right;
      state->latest.stamp = std::chrono::steady_clock::now();
      state->latest.driver_error = driver_error;
      state->latest.has_update = true;
    }

    const auto elapsed = std::chrono::steady_clock::now() - loop_start;
    if (elapsed < period) {
      std::this_thread::sleep_for(period - elapsed);
    }
  }

  if (driver) {
    driver->stop();
  }
}

void Vl53MuxNode::on_timer()
{
  publish_latest();
}

void Vl53MuxNode::publish_latest()
{
  if (!worker_state_) {
    publish_distance(left_pub_, std::nullopt);
    publish_distance(right_pub_, std::nullopt);
    return;
  }

  Vl53LatestState latest;
  {
    std::lock_guard<std::mutex> lock(worker_state_->mutex);
    latest = worker_state_->latest;
  }

  if (!latest.has_update) {
    publish_distance(left_pub_, std::nullopt);
    publish_distance(right_pub_, std::nullopt);

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "VL53 no worker update yet");
    return;
  }

  if (latest_is_stale(latest)) {
    publish_distance(left_pub_, std::nullopt);
    publish_distance(right_pub_, std::nullopt);

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "VL53 worker data stale: driver_error='%s'",
      latest.driver_error.c_str());
    return;
  }

  publish_reading(left_pub_, latest.left);
  publish_reading(right_pub_, latest.right);

  if (!latest.left.valid || !latest.right.valid) {
    const auto left_text = reading_text(latest.left);
    const auto right_text = reading_text(latest.right);

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "VL53 read warning: left={%s} right={%s} driver_error='%s'",
      left_text.c_str(),
      right_text.c_str(),
      latest.driver_error.c_str());
  }
}

void Vl53MuxNode::publish_reading(
  const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher,
  const Vl53l1xReading & reading)
{
  if (!reading.valid) {
    publish_distance(publisher, std::nullopt);
    return;
  }

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

bool Vl53MuxNode::latest_is_stale(const Vl53LatestState & latest) const
{
  const auto age = std::chrono::steady_clock::now() - latest.stamp;
  return age > std::chrono::duration<double>(config_.stale_timeout_s);
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
