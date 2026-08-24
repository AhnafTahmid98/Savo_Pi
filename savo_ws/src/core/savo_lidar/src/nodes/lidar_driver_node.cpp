#include "savo_lidar/diagnostics.hpp"
#include "savo_lidar/rplidar_driver.hpp"
#include "savo_lidar/scan_acquisition_worker.hpp"
#include "savo_lidar/scan_publisher.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
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

    heartbeat_timer_ = create_wall_timer(
      period_from_hz(heartbeat_hz_),
      [this]() {
        publish_heartbeat();
      });

    if (auto_start_) {
      acquisition_worker_ = std::make_unique<ScanAcquisitionWorker>(
        [this]() {return acquire_scan();},
        [this](const LidarScan & scan) {publish_scan(scan);},
        [this](const std::string & message) {return handle_acquisition_error(message);},
        std::chrono::duration<double>(config_.serial.reconnect_delay_s),
        [this]() {
          if (driver_) {
            driver_->cancel_pending_operation();
          }
        });
      acquisition_worker_->start();
    }

    RCLCPP_INFO(
      get_logger(),
      "Robot Savo C++ LiDAR driver node ready | port=%s | scan_topic=%s",
      config_.serial.port.c_str(),
      config_.scan_topic.c_str());
  }

  ~LidarDriverNode() override
  {
    if (acquisition_worker_) {
      acquisition_worker_->request_stop();
      acquisition_worker_->join();
    }

    if (driver_) {
      driver_->stop();
    }

    driver_running_.store(false);
  }

private:
  void load_parameters()
  {
    config_.serial.port = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    config_.serial.baudrate = declare_parameter<int>("baudrate", 115200);
    config_.serial.timeout_s = declare_parameter<double>("serial_timeout_s", 1.0);
    config_.serial.dtr_enable = declare_parameter<bool>("dtr_enable", false);
    config_.serial.rts_enable = declare_parameter<bool>("rts_enable", false);
    config_.serial.reconnect_on_error = declare_parameter<bool>("reconnect_on_error", true);
    config_.serial.reconnect_delay_s = declare_parameter<double>("reconnect_delay_s", 0.5);
    config_.serial.max_reconnect_delay_s =
      declare_parameter<double>("max_reconnect_delay_s", 5.0);

    config_.frame_id = declare_parameter<std::string>("frame_id", "laser_frame");
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

    heartbeat_hz_ = declare_parameter<double>("heartbeat_hz", 1.0);
    read_timeout_s_ = declare_parameter<double>("read_timeout_s", 2.0);
    auto_start_ = declare_parameter<bool>("auto_start", true);

    config_.validate();

    if (heartbeat_hz_ <= 0.0) {
      throw std::invalid_argument("heartbeat_hz must be > 0");
    }

    if (read_timeout_s_ <= 0.0) {
      throw std::invalid_argument("read_timeout_s must be > 0");
    }
  }

  void start_driver()
  {
    driver_->start();
    driver_running_.store(true);
    set_last_error("");

    RCLCPP_INFO(
      get_logger(),
      "RPLIDAR driver started | port=%s | baudrate=%d",
      config_.serial.port.c_str(),
      config_.serial.baudrate);
  }

  LidarScan acquire_scan()
  {
    if (!driver_) {
      throw std::runtime_error("driver object is not initialized");
    }

    if (!driver_->running()) {
      start_driver();
    }

    return driver_->read_scan(
      read_timeout_s_,
      [this]() {return now().nanoseconds();});
  }

  void publish_scan(const LidarScan & scan)
  {
    scan_publisher_->publish(scan);

    const auto published_at = std::chrono::steady_clock::now();
    double scan_rate_hz =
      scan.scan_time_s > 0.0 ? 1.0 / scan.scan_time_s : 0.0;

    if (last_publish_time_ != std::chrono::steady_clock::time_point{}) {
      const auto publish_period_s =
        std::chrono::duration<double>(published_at - last_publish_time_).count();
      if (publish_period_s > 0.0) {
        scan_rate_hz = 1.0 / publish_period_s;
      }
    }

    last_publish_time_ = published_at;
    last_scan_rate_hz_ = scan_rate_hz;
    scan_count_.store(driver_->scan_count());

    const auto diagnostics = make_ok_driver_diagnostics(
      scan,
      scan_count_.load(),
      last_scan_rate_hz_,
      config_.frame_id,
      config_.scan_topic,
      config_.serial.port);

    publish_driver_state(diagnostics);
  }

  bool handle_acquisition_error(const std::string & message)
  {
    driver_->stop();
    driver_running_.store(false);
    last_publish_time_ = std::chrono::steady_clock::time_point{};
    last_scan_rate_hz_ = 0.0;
    set_last_error(message);
    publish_error_state(message);

    RCLCPP_WARN(
      get_logger(),
      "RPLIDAR acquisition failed: %s",
      message.c_str());

    return config_.serial.reconnect_on_error;
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
      driver_running_.load(),
      scan_count_.load(),
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

    const bool running = driver_running_.load();

    const auto status = running ? std::string(STATUS_OK) : std::string(STATUS_OFFLINE);
    const auto message = running ? "driver running" : "driver not running";
    const auto scan_count = scan_count_.load();
    const auto last_error = get_last_error();

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

  void set_last_error(const std::string & message)
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
  }

  std::string get_last_error() const
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
  }

  RplidarConfig config_;

  std::string state_topic_{"/savo_lidar/state"};
  std::string heartbeat_topic_{"/savo_lidar/heartbeat"};

  double heartbeat_hz_{1.0};
  double read_timeout_s_{2.0};
  bool auto_start_{true};

  std::chrono::steady_clock::time_point last_publish_time_{};
  double last_scan_rate_hz_{0.0};

  std::atomic<bool> driver_running_{false};
  std::atomic<std::uint64_t> scan_count_{0U};
  mutable std::mutex error_mutex_;
  std::string last_error_;

  std::unique_ptr<RplidarDriver> driver_;
  std::unique_ptr<ScanAcquisitionWorker> acquisition_worker_;
  std::unique_ptr<ScanPublisher> scan_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;

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
