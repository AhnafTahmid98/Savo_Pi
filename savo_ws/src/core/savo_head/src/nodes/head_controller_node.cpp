#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "savo_head/core/diagnostics.hpp"
#include "savo_head/drivers/pantilt_driver.hpp"

namespace savo_head
{

namespace
{

bool close_mode(double value, double target)
{
  return std::isfinite(value) && std::abs(value - target) < 1e-6;
}


constexpr double kCmdAbsolute = 0.0;
constexpr double kCmdDelta = 1.0;
constexpr double kCmdCenter = 2.0;
constexpr double kCmdHold = 3.0;
constexpr double kCmdStop = 4.0;

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, int value)
{
  return kv(key, std::to_string(value));
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, bool value)
{
  return kv(key, std::string(value ? "true" : "false"));
}

std::uint8_t ros_level(DiagnosticLevel level)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  switch (level) {
    case DiagnosticLevel::kOk:
      return DiagnosticStatus::OK;
    case DiagnosticLevel::kWarn:
      return DiagnosticStatus::WARN;
    case DiagnosticLevel::kError:
      return DiagnosticStatus::ERROR;
  }

  return DiagnosticStatus::ERROR;
}

}  // namespace

class HeadControllerNode : public rclcpp::Node
{
public:
  HeadControllerNode()
  : Node("head_controller_node")
  {
    declare_parameters();
    driver_ = make_driver();

    try {
      driver_->open();
      RCLCPP_INFO(
        get_logger(),
        "head controller started with backend=%s",
        driver_->config().backend.c_str());
    } catch (const std::exception & exc) {
      last_error_ = exc.what();
      RCLCPP_ERROR(get_logger(), "failed to open head driver: %s", exc.what());
    }

    last_cmd_time_s_ = now_s();

    pan_tilt_cmd_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      get_parameter("pan_tilt_cmd_topic").as_string(),
      rclcpp::QoS(10).reliable(),
      std::bind(&HeadControllerNode::on_pan_tilt_cmd, this, std::placeholders::_1));

    emergency_center_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("emergency_center_topic").as_string(),
      rclcpp::QoS(10).reliable(),
      std::bind(&HeadControllerNode::on_emergency_center, this, std::placeholders::_1));

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      get_parameter("pan_tilt_state_topic").as_string(),
      rclcpp::QoS(10).reliable());

    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("status_topic").as_string(),
      rclcpp::QoS(10).reliable());

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("diagnostics_topic").as_string(),
      rclcpp::QoS(10).reliable());

    dashboard_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("dashboard_text_topic").as_string(),
      rclcpp::QoS(10).reliable());

    center_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("center_service").as_string(),
      std::bind(
        &HeadControllerNode::on_center_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    health_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("health_check_service").as_string(),
      std::bind(
        &HeadControllerNode::on_health_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    const auto control_hz = std::max(1.0, get_parameter("control_hz").as_double());
    const auto status_hz = std::max(0.2, get_parameter("status_hz").as_double());

    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_hz),
      std::bind(&HeadControllerNode::publish_state, this));

    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_hz),
      std::bind(&HeadControllerNode::publish_status, this));
  }

  ~HeadControllerNode() override
  {
    shutdown_driver();
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("backend", kHeadBackendPca9685);
    declare_parameter<int>("i2c_bus", kI2cBusDefault);
    declare_parameter<int>("pca9685_address", kPca9685AddressDefault);
    declare_parameter<double>("pwm_frequency_hz", kPca9685PwmFrequencyHzDefault);

    declare_parameter<std::string>("pan_logical_channel", kPanLogicalChannel);
    declare_parameter<std::string>("tilt_logical_channel", kTiltLogicalChannel);
    declare_parameter<int>("pan_pca9685_channel", kPanPca9685Channel);
    declare_parameter<int>("tilt_pca9685_channel", kTiltPca9685Channel);

    declare_parameter<int>("pan_min_deg", kPanMinDeg);
    declare_parameter<int>("pan_center_deg", kPanCenterDeg);
    declare_parameter<int>("pan_max_deg", kPanMaxDeg);

    declare_parameter<int>("tilt_min_deg", kTiltMinDeg);
    declare_parameter<int>("tilt_center_deg", kTiltCenterDeg);
    declare_parameter<int>("tilt_max_deg", kTiltMaxDeg);

    declare_parameter<int>("servo_error_deg", kServoErrorDegDefault);

    declare_parameter<bool>("center_on_start", false);
    declare_parameter<bool>("center_on_shutdown", true);
    declare_parameter<bool>("stop_on_watchdog_timeout", true);

    declare_parameter<double>("control_hz", kControlHz);
    declare_parameter<double>("status_hz", kStatusHz);
    declare_parameter<double>("watchdog_timeout_s", kWatchdogTimeoutS);

    declare_parameter<std::string>("pan_tilt_cmd_topic", kTopicPanTiltCmd);
    declare_parameter<std::string>("pan_tilt_state_topic", kTopicPanTiltState);
    declare_parameter<std::string>("status_topic", kTopicStatus);
    declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
    declare_parameter<std::string>("dashboard_text_topic", kTopicDashboardText);
    declare_parameter<std::string>("emergency_center_topic", kTopicEmergencyCenter);

    declare_parameter<std::string>("center_service", "/savo_head/center");
    declare_parameter<std::string>("health_check_service", "/savo_head/health_check");
  }

  std::unique_ptr<PanTiltDriver> make_driver()
  {
    PanTiltDriverConfig config;
    config.backend = get_parameter("backend").as_string();
    config.i2c_bus = get_parameter("i2c_bus").as_int();
    config.pca9685_address = get_parameter("pca9685_address").as_int();
    config.pwm_frequency_hz = get_parameter("pwm_frequency_hz").as_double();
    config.center_on_open = get_parameter("center_on_start").as_bool();
    config.center_on_close = false;

    config.calibration.pan.logical_channel = get_parameter("pan_logical_channel").as_string();
    config.calibration.tilt.logical_channel = get_parameter("tilt_logical_channel").as_string();
    config.calibration.pan.pca9685_channel = get_parameter("pan_pca9685_channel").as_int();
    config.calibration.tilt.pca9685_channel = get_parameter("tilt_pca9685_channel").as_int();

    config.calibration.pan.min_deg = get_parameter("pan_min_deg").as_int();
    config.calibration.pan.center_deg = get_parameter("pan_center_deg").as_int();
    config.calibration.pan.max_deg = get_parameter("pan_max_deg").as_int();

    config.calibration.tilt.min_deg = get_parameter("tilt_min_deg").as_int();
    config.calibration.tilt.center_deg = get_parameter("tilt_center_deg").as_int();
    config.calibration.tilt.max_deg = get_parameter("tilt_max_deg").as_int();

    const auto servo_error = get_parameter("servo_error_deg").as_int();
    config.calibration.pan.error_deg = servo_error;
    config.calibration.tilt.error_deg = servo_error;

    config = config.normalized();

    if (!valid_head_backend(config.backend)) {
      RCLCPP_WARN(get_logger(), "invalid backend, falling back to dryrun");
      config.backend = kHeadBackendDryrun;
    }

    const auto errors = config.calibration.validation_errors();
    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR(get_logger(), "head calibration error: %s", error.c_str());
      }
      throw std::runtime_error("invalid head servo calibration");
    }

    return std::make_unique<PanTiltDriver>(config);
  }

  void on_pan_tilt_cmd(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    const auto stamp = now_s();
    last_cmd_time_s_ = stamp;

    try {
      const auto command = vector_to_command(*msg, stamp);
      const auto state = driver_->apply_command(command);
      last_error_.reset();

      RCLCPP_DEBUG(
        get_logger(),
        "pan_tilt command applied pan=%d tilt=%d mode=%s",
        state.pan_deg,
        state.tilt_deg,
        to_string(state.mode));
    } catch (const std::exception & exc) {
      last_error_ = exc.what();
      RCLCPP_ERROR(get_logger(), "pan/tilt command failed: %s", exc.what());
    }
  }

  void on_emergency_center(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto stamp = now_s();
    last_cmd_time_s_ = stamp;

    try {
      const auto reason = msg->data.empty() ? "emergency_center" : msg->data;
      const auto state = driver_->apply_command(
        center_command(stamp, CommandSource::kEmergency, reason));
      last_error_.reset();

      RCLCPP_WARN(
        get_logger(),
        "emergency center applied pan=%d tilt=%d",
        state.pan_deg,
        state.tilt_deg);
    } catch (const std::exception & exc) {
      last_error_ = exc.what();
      RCLCPP_ERROR(get_logger(), "emergency center failed: %s", exc.what());
    }
  }

  void on_center_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    try {
      const auto state = driver_->apply_command(
        center_command(now_s(), CommandSource::kService, "center_service"));
      last_error_.reset();

      response->success = true;
      response->message =
        "centered pan=" + std::to_string(state.pan_deg) + " tilt=" + std::to_string(state.tilt_deg);
    } catch (const std::exception & exc) {
      last_error_ = exc.what();
      response->success = false;
      response->message = exc.what();
    }
  }

  void on_health_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    const auto health = current_health();
    response->success = health.ok;
    response->message = health.message + " " + driver_->status_text();
  }

  PanTiltCommand vector_to_command(const geometry_msgs::msg::Vector3 & msg, double stamp_s) const
  {
    if (close_mode(msg.z, kCmdDelta)) {
      return delta_command(
        static_cast<int>(std::llround(msg.x)),
        static_cast<int>(std::llround(msg.y)),
        stamp_s,
        CommandSource::kTopic,
        "vector_delta");
    }

    if (close_mode(msg.z, kCmdCenter)) {
      return center_command(stamp_s, CommandSource::kTopic, "vector_center");
    }

    if (close_mode(msg.z, kCmdHold)) {
      return hold_command(stamp_s, CommandSource::kTopic, "vector_hold");
    }

    if (close_mode(msg.z, kCmdStop)) {
      return stop_command(stamp_s, CommandSource::kTopic, "vector_stop");
    }

    return absolute_command(
      static_cast<int>(std::llround(msg.x)),
      static_cast<int>(std::llround(msg.y)),
      stamp_s,
      CommandSource::kTopic,
      "vector_absolute");
  }

  void publish_state()
  {
    watchdog_check();

    const auto state = driver_->state();

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {kJointPan, kJointTilt};
    msg.position = {
      degrees_to_radians(state.pan_deg),
      degrees_to_radians(state.tilt_deg)
    };

    joint_pub_->publish(msg);
  }

  void watchdog_check()
  {
    if (!get_parameter("stop_on_watchdog_timeout").as_bool()) {
      return;
    }

    const auto timeout_s = get_parameter("watchdog_timeout_s").as_double();
    const auto age_s = now_s() - last_cmd_time_s_;

    if (age_s <= timeout_s) {
      return;
    }

    if (driver_->state().source == to_string(CommandSource::kWatchdog)) {
      return;
    }

    try {
      const auto state = driver_->apply_command(
        hold_command(now_s(), CommandSource::kWatchdog, "watchdog_hold"));
      (void)state;
    } catch (const std::exception & exc) {
      last_error_ = exc.what();
    }
  }

  void publish_status()
  {
    const auto health = current_health();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "savo_head.head_controller";
    status.hardware_id = "savo_head";
    status.level = ros_level(health.level);
    status.message = health.message;

    const auto state = driver_->state();
    status.values = {
      kv("backend", driver_->config().backend),
      kv("opened", driver_->opened()),
      kv("dryrun", driver_->dryrun()),
      kv("pan_deg", state.pan_deg),
      kv("tilt_deg", state.tilt_deg),
      kv("mode", to_string(state.mode)),
      kv("state_status", to_string(state.status)),
      kv("source", state.source),
      kv("last_error", last_error_.value_or(""))
    };

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status = {status};

    status_pub_->publish(array);
    diagnostics_pub_->publish(array);

    std_msgs::msg::String dashboard;
    dashboard.data = dashboard_text(health) + " " + driver_->status_text();
    dashboard_pub_->publish(dashboard);
  }

  HeadHealthSummary current_health() const
  {
    const auto hardware = hardware_health(
      driver_ && driver_->opened(),
      driver_ && driver_->dryrun(),
      last_error_);

    const auto pan_tilt = pan_tilt_health(
      driver_->state(),
      now_s(),
      get_parameter("watchdog_timeout_s").as_double() * 2.0,
      true);

    return summarize_health({hardware, pan_tilt}, true);
  }

  void shutdown_driver()
  {
    if (!driver_) {
      return;
    }

    try {
      if (get_parameter("center_on_shutdown").as_bool()) {
        const auto state = driver_->apply_command(
          center_command(now_s(), CommandSource::kSystem, "shutdown_center"));
        (void)state;
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "could not center head on shutdown: %s", exc.what());
    }

    try {
      driver_->close();
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "could not close head driver: %s", exc.what());
    }
  }

  double now_s() const
  {
    return now().seconds();
  }

  static double degrees_to_radians(int degrees)
  {
    return static_cast<double>(degrees) * M_PI / 180.0;
  }

  std::unique_ptr<PanTiltDriver> driver_{};
  std::optional<std::string> last_error_{};
  double last_cmd_time_s_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pan_tilt_cmd_sub_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_center_sub_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dashboard_pub_{};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr center_service_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr health_service_{};

  rclcpp::TimerBase::SharedPtr state_timer_{};
  rclcpp::TimerBase::SharedPtr status_timer_{};
};

}  // namespace savo_head

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<savo_head::HeadControllerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    std::cerr << "head_controller_node failed: " << exc.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
