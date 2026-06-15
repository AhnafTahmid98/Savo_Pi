#include "savo_lidar/diagnostics.hpp"
#include "savo_lidar/rplidar_driver.hpp"
#include "savo_lidar/scan_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace savo_lidar
{
namespace
{

std::chrono::milliseconds period_from_hz(double hz)
{
  if (hz <= 0.0) {
    hz = 1.0;
  }

  const auto period_ms = static_cast<int>(1000.0 / hz);
  return std::chrono::milliseconds(std::max(1, period_ms));
}

}  // namespace

class LidarDriverNode final : public rclcpp::Node
{
public:
  explicit LidarDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lidar_driver_node", options)
  {
    load_parameters();

    driver_ = std::make_unique<RplidarDriver>(config_);

    scan_publisher_ = std::make_unique<ScanPublisher>(
      this,
      config_.scan_topic,
      rclcpp::SensorDataQoS());

    state_publisher_ = create_publisher<std_msgs::msg::String>(
      state_topic_,
      rclcpp::QoS(10).reliable());

    heartbeat_publisher_ = create_publisher<std_msgs::msg::String>(
      heartbeat_topic_,
      rclcpp::QoS(1).reliable().transient_local());

    scan_timer_ = create_wall_timer(
      period_from_hz(publish_rate_hz_),
      [this]() {
        on_scan_timer();
      });

    heartbeat_timer_ = create_wall_timer(
      period_from_hz(heartbeat_hz_),
      [this]() {
        publish_heartbeat();
      });

    if (auto_start_) {
      start_driver();
    }

    RCLCPP_INFO(
      get_logger(),
      "Robot Savo C++ LiDAR driver node ready | port=%s | scan_topic=%s",
      config_.serial.port.c_str(),
      config_.scan_topic.c_str());
  }

  ~LidarDriverNode() override
  {
    if (driver_) {
      driver_->stop();
    }
  }

private:
  void load_parameters()
  {
    config_.serial.port = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    config_.serial.baudrate = declare_parameter<int>("baudrate", 115200);
    config_.serial.timeout_s = declare_parameter<double>("serial_timeout_s", 1.0);
    config_.serial.reconnect_on_error = declare_parameter<bool>("reconnect_on_error", true);
    config_.serial.reconnect_delay_s = declare_parameter<double>("reconnect_delay_s", 0.5);
    config_.serial.max_reconnect_delay_s =
      declare_parameter<double>("max_reconnect_delay_s", 5.0);

    config_.frame_id = declare_parameter<std::string>("frame_id", "laser");
    config_.scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    config_.scan_mode = declare_parameter<std::string>("scan_mode", "standard");

    config_.expected_scan_rate_hz = declare_parameter<double>("expected_scan_rate_hz", 5.5);
    config_.motor_start_settle_s = declare_parameter<double>("motor_start_settle_s", 1.0);
    config_.motor_stop_timeout_s = declare_parameter<double>("motor_stop_timeout_s", 1.0);

    config_.min_range_m = static_cast<float>(declare_parameter<double>("min_range_m", 0.15));
    config_.max_range_m = static_cast<float>(declare_parameter<double>("max_range_m", 12.0));

    config_.inverted = declare_parameter<bool>("inverted", false);
    config_.angle_offset_rad = declare_parameter<double>("angle_offset_rad", 0.0);

    state_topic_ = declare_parameter<std::string>("driver_state_topic", "/savo_lidar/state");
    heartbeat_topic_ =
      declare_parameter<std::string>("heartbeat_topic", "/savo_lidar/heartbeat");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.5);
    heartbeat_hz_ = declare_parameter<double>("heartbeat_hz", 1.0);
    read_timeout_s_ = declare_parameter<double>("read_timeout_s", 2.0);
    auto_start_ = declare_parameter<bool>("auto_start", true);

    config_.validate();

    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be > 0");
    }

    if (heartbeat_hz_ <= 0.0) {
      throw std::invalid_argument("heartbeat_hz must be > 0");
    }

    if (read_timeout_s_ <= 0.0) {
      throw std::invalid_argument("read_timeout_s must be > 0");
    }
  }

  void start_driver()
  {
    last_start_attempt_ = now();

    try {
      driver_->start();

      RCLCPP_INFO(
        get_logger(),
        "RPLIDAR driver started | port=%s | baudrate=%d",
        config_.serial.port.c_str(),
        config_.serial.baudrate);
    } catch (const std::exception & exc) {
      publish_error_state(exc.what());

      RCLCPP_WARN(
        get_logger(),
        "RPLIDAR driver start failed: %s",
        exc.what());
    }
  }

  void on_scan_timer()
  {
    if (!driver_) {
      publish_error_state("driver object is not initialized");
      return;
    }

    if (!driver_->running()) {
      maybe_reconnect();
      return;
    }

    try {
      auto scan = driver_->read_scan(read_timeout_s_);

      scan_publisher_->publish(scan);

      const auto now_time = now();
      double scan_rate_hz = 0.0;

      if (last_scan_time_.nanoseconds() > 0) {
        const auto dt_s = (now_time - last_scan_time_).seconds();
        if (dt_s > 0.0) {
          scan_rate_hz = 1.0 / dt_s;
        }
      }

      last_scan_time_ = now_time;
      last_scan_rate_hz_ = scan_rate_hz > 0.0 ? scan_rate_hz : config_.expected_scan_rate_hz;

      const auto diagnostics = make_ok_driver_diagnostics(
        scan,
        driver_->scan_count(),
        last_scan_rate_hz_,
        config_.frame_id,
        config_.scan_topic,
        config_.serial.port);

      publish_driver_state(diagnostics);
    } catch (const std::exception & exc) {
      publish_error_state(exc.what());

      RCLCPP_WARN(
        get_logger(),
        "RPLIDAR scan read failed: %s",
        exc.what());

      driver_->stop();
    }
  }

  void maybe_reconnect()
  {
    if (!config_.serial.reconnect_on_error) {
      publish_error_state("driver is not running");
      return;
    }

    const auto now_time = now();

    if (last_start_attempt_.nanoseconds() > 0) {
      const auto age_s = (now_time - last_start_attempt_).seconds();
      if (age_s < config_.serial.reconnect_delay_s) {
        return;
      }
    }

    start_driver();
  }

  void publish_driver_state(const DriverDiagnostics & diagnostics)
  {
    std_msgs::msg::String msg;
    msg.data = driver_diagnostics_to_json(diagnostics);
    state_publisher_->publish(msg);
  }

  void publish_error_state(const std::string & message)
  {
    const auto diagnostics = make_error_driver_diagnostics(
      message,
      false,
      driver_ && driver_->running(),
      driver_ ? driver_->scan_count() : 0U,
      config_.frame_id,
      config_.scan_topic,
      config_.serial.port);

    publish_driver_state(diagnostics);
  }

  void publish_heartbeat()
  {
    if (!heartbeat_publisher_) {
      return;
    }

    const bool running = driver_ && driver_->running();

    const auto status = running ? std::string(STATUS_OK) : std::string(STATUS_OFFLINE);
    const auto message = running ? "driver running" : "driver not running";
    const auto scan_count = driver_ ? driver_->scan_count() : 0U;
    const auto last_error = driver_ ? driver_->last_error() : std::string();

    std_msgs::msg::String msg;
    msg.data = make_driver_heartbeat_json(
      "lidar_driver_node",
      status,
      message,
      running,
      scan_count,
      last_error);

    heartbeat_publisher_->publish(msg);
  }

  RplidarConfig config_;

  std::string state_topic_{"/savo_lidar/state"};
  std::string heartbeat_topic_{"/savo_lidar/heartbeat"};

  double publish_rate_hz_{5.5};
  double heartbeat_hz_{1.0};
  double read_timeout_s_{2.0};
  bool auto_start_{true};

  rclcpp::Time last_scan_time_;
  rclcpp::Time last_start_attempt_;
  double last_scan_rate_hz_{0.0};

  std::unique_ptr<RplidarDriver> driver_;
  std::unique_ptr<ScanPublisher> scan_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;

  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace savo_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<savo_lidar::LidarDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lidar_driver_node"),
      "Fatal error in C++ LiDAR driver node: %s",
      exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}