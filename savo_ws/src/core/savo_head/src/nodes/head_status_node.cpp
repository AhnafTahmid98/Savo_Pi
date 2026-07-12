#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "savo_head/core/diagnostics.hpp"
#include "savo_head/core/head_types.hpp"

namespace savo_head
{

namespace
{

struct TopicWatch
{
  std::string name;
  double last_stamp_s{0.0};
  std::string last_text{};
  bool seen{false};

  void update(double stamp_s, std::string text = {})
  {
    last_stamp_s = stamp_s;
    last_text = std::move(text);
    seen = true;
  }

  [[nodiscard]] double age_s(double now_s) const
  {
    if (!seen || last_stamp_s <= 0.0) {
      return 0.0;
    }

    return std::max(0.0, now_s - last_stamp_s);
  }

  [[nodiscard]] HeadStatus state(double now_s, double stale_timeout_s, bool required) const
  {
    if (!seen) {
      return required ? HeadStatus::kStale : HeadStatus::kDryrun;
    }

    if (age_s(now_s) > stale_timeout_s) {
      return required ? HeadStatus::kStale : HeadStatus::kDryrun;
    }

    return HeadStatus::kOk;
  }
};

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, double value)
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

std::chrono::nanoseconds seconds_to_period(double seconds)
{
  const auto safe_seconds = std::max(0.001, seconds);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(safe_seconds));
}

ComponentHealth watch_health(
  const TopicWatch & watch,
  double now_s,
  double stale_timeout_s,
  bool required)
{
  const auto status = watch.state(now_s, stale_timeout_s, required);

  ComponentHealth health = make_component_health(
    "savo_head." + watch.name,
    status,
    to_string(status));

  health.add_value("seen", watch.seen);
  health.add_value("required", required);
  health.add_value("age_s", watch.age_s(now_s));
  health.add_value("last_text", watch.last_text);

  return health;
}

}  // namespace

class HeadStatusNode : public rclcpp::Node
{
public:
  HeadStatusNode()
  : Node("head_status_node"),
    pan_tilt_watch_{"pan_tilt"},
    scan_watch_{"scan"},
    camera_watch_{"camera"},
    semantic_watch_{"semantic"}
  {
    declare_parameters();

    pan_tilt_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("pan_tilt_state_topic").as_string(),
      rclcpp::QoS(10).reliable(),
      std::bind(&HeadStatusNode::on_pan_tilt_state, this, std::placeholders::_1));

    scan_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("scan_state_topic").as_string(),
      rclcpp::QoS(10).reliable(),
      std::bind(&HeadStatusNode::on_scan_state, this, std::placeholders::_1));

    camera_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("camera_status_topic").as_string(),
      rclcpp::QoS(10).best_effort(),
      std::bind(&HeadStatusNode::on_camera_status, this, std::placeholders::_1));

    semantic_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("semantic_confirmations_topic").as_string(),
      rclcpp::QoS(10).reliable(),
      std::bind(&HeadStatusNode::on_semantic_confirmation, this, std::placeholders::_1));

    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("status_topic").as_string(),
      rclcpp::QoS(10).reliable());

    dashboard_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("dashboard_text_topic").as_string(),
      rclcpp::QoS(10).reliable());

    health_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("health_check_service").as_string(),
      std::bind(
        &HeadStatusNode::on_health_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    const auto status_hz = std::max(0.2, get_parameter("status_publish_hz").as_double());

    status_timer_ = create_wall_timer(
      seconds_to_period(1.0 / status_hz),
      std::bind(&HeadStatusNode::publish_status, this));

    RCLCPP_INFO(get_logger(), "head status node started");
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("pan_tilt_state_topic", kTopicPanTiltState);
    declare_parameter<std::string>("scan_state_topic", kTopicScanState);
    declare_parameter<std::string>("camera_status_topic", kTopicCameraStatus);
    declare_parameter<std::string>("semantic_confirmations_topic", kTopicSemanticConfirmations);

    declare_parameter<std::string>("status_topic", kTopicStatus);
    declare_parameter<std::string>("dashboard_text_topic", kTopicDashboardText);
    declare_parameter<std::string>("health_check_service", "/savo_head/health_check_status");

    declare_parameter<double>("status_publish_hz", kStatusHz);
    declare_parameter<double>("state_stale_timeout_s", 0.50);
    declare_parameter<double>("scan_stale_timeout_s", 1.00);
    declare_parameter<double>("camera_stream_stale_timeout_s", 2.00);
    declare_parameter<double>("apriltag_detection_stale_timeout_s", 1.00);

    declare_parameter<bool>("require_pan_tilt_for_ok", true);
    declare_parameter<bool>("require_scan_for_ok", false);
    declare_parameter<bool>("require_camera_for_ok", false);
    declare_parameter<bool>("require_apriltag_for_ok", false);
  }

  void on_pan_tilt_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = *msg;
    pan_tilt_watch_.update(now_s(), joint_text(*msg));
    last_error_.reset();
  }

  void on_scan_state(const std_msgs::msg::String::SharedPtr msg)
  {
    scan_watch_.update(now_s(), msg->data);
    last_error_.reset();
  }

  void on_camera_status(const std_msgs::msg::String::SharedPtr msg)
  {
    camera_watch_.update(now_s(), msg->data);
    last_error_.reset();
  }

  void on_semantic_confirmation(const std_msgs::msg::String::SharedPtr msg)
  {
    semantic_watch_.update(now_s(), msg->data);
    last_error_.reset();
  }

  void on_health_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    const auto health = current_health();
    response->success = health.ok;
    response->message = health.message + " " + component_text();
  }

  void publish_status()
  {
    try {
      const auto health = current_health();

      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "savo_head.head_status";
      status.hardware_id = "savo_head";
      status.level = ros_level(health.level);
      status.message = health.message;

      const auto now_value = now_s();

      status.values = {
        kv("pan_tilt_state", to_string(pan_tilt_watch_.state(
          now_value,
          get_parameter("state_stale_timeout_s").as_double(),
          get_parameter("require_pan_tilt_for_ok").as_bool()))),
        kv("scan_state", to_string(scan_watch_.state(
          now_value,
          get_parameter("scan_stale_timeout_s").as_double(),
          get_parameter("require_scan_for_ok").as_bool()))),
        kv("camera_state", to_string(camera_watch_.state(
          now_value,
          get_parameter("camera_stream_stale_timeout_s").as_double(),
          get_parameter("require_camera_for_ok").as_bool()))),
        kv("semantic_state", to_string(semantic_watch_.state(
          now_value,
          get_parameter("apriltag_detection_stale_timeout_s").as_double(),
          get_parameter("require_apriltag_for_ok").as_bool()))),
        kv("pan_tilt_age_s", pan_tilt_watch_.age_s(now_value)),
        kv("scan_age_s", scan_watch_.age_s(now_value)),
        kv("camera_age_s", camera_watch_.age_s(now_value)),
        kv("semantic_age_s", semantic_watch_.age_s(now_value)),
        kv("pan_tilt_text", pan_tilt_watch_.last_text),
        kv("scan_text", scan_watch_.last_text),
        kv("camera_text", camera_watch_.last_text),
        kv("semantic_text", semantic_watch_.last_text),
        kv("last_error", last_error_.value_or(""))
      };

      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = now();
      array.status = {status};

      status_pub_->publish(array);

      std_msgs::msg::String dashboard;
      dashboard.data = dashboard_text_line(health);
      dashboard_pub_->publish(dashboard);
    } catch (const std::exception & exc) {
      last_error_ = exc.what();
      RCLCPP_ERROR(get_logger(), "status publish failed: %s", exc.what());
    }
  }

  HeadHealthSummary current_health() const
  {
    const auto now_value = now_s();

    std::vector<ComponentHealth> components;
    components.push_back(
      watch_health(
        pan_tilt_watch_,
        now_value,
        get_parameter("state_stale_timeout_s").as_double(),
        get_parameter("require_pan_tilt_for_ok").as_bool()));

    components.push_back(
      watch_health(
        scan_watch_,
        now_value,
        get_parameter("scan_stale_timeout_s").as_double(),
        get_parameter("require_scan_for_ok").as_bool()));

    components.push_back(
      watch_health(
        camera_watch_,
        now_value,
        get_parameter("camera_stream_stale_timeout_s").as_double(),
        get_parameter("require_camera_for_ok").as_bool()));

    components.push_back(
      watch_health(
        semantic_watch_,
        now_value,
        get_parameter("apriltag_detection_stale_timeout_s").as_double(),
        get_parameter("require_apriltag_for_ok").as_bool()));

    if (last_error_.has_value() && !last_error_->empty()) {
      components.push_back(error_component("savo_head.status", *last_error_));
    }

    return summarize_health(components, false);
  }

  std::string dashboard_text_line(const HeadHealthSummary & health) const
  {
    std::ostringstream stream;
    stream << "savo_head status=" << health.message << " "
           << component_text();

    return stream.str();
  }

  std::string component_text() const
  {
    const auto now_value = now_s();

    std::ostringstream stream;
    stream << "pan_tilt=" << to_string(pan_tilt_watch_.state(
        now_value,
        get_parameter("state_stale_timeout_s").as_double(),
        get_parameter("require_pan_tilt_for_ok").as_bool()))
           << " scan=" << to_string(scan_watch_.state(
        now_value,
        get_parameter("scan_stale_timeout_s").as_double(),
        get_parameter("require_scan_for_ok").as_bool()))
           << " camera=" << to_string(camera_watch_.state(
        now_value,
        get_parameter("camera_stream_stale_timeout_s").as_double(),
        get_parameter("require_camera_for_ok").as_bool()))
           << " semantic=" << to_string(semantic_watch_.state(
        now_value,
        get_parameter("apriltag_detection_stale_timeout_s").as_double(),
        get_parameter("require_apriltag_for_ok").as_bool()));

    return stream.str();
  }

  static std::string joint_text(const sensor_msgs::msg::JointState & msg)
  {
    std::ostringstream stream;

    if (msg.position.size() >= 2U) {
      stream << "pan_rad=" << msg.position[0] << ";tilt_rad=" << msg.position[1];
    } else if (msg.position.size() == 1U) {
      stream << "pan_rad=" << msg.position[0];
    } else {
      stream << "joint_state_empty";
    }

    return stream.str();
  }

  double now_s() const
  {
    return now().seconds();
  }

  TopicWatch pan_tilt_watch_;
  TopicWatch scan_watch_;
  TopicWatch camera_watch_;
  TopicWatch semantic_watch_;

  std::optional<sensor_msgs::msg::JointState> last_joint_state_{};
  std::optional<std::string> last_error_{};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pan_tilt_sub_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scan_sub_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_sub_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr semantic_sub_{};

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dashboard_pub_{};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr health_service_{};
  rclcpp::TimerBase::SharedPtr status_timer_{};
};

}  // namespace savo_head

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<savo_head::HeadStatusNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    std::cerr << "head_status_node failed: " << exc.what() << std::endl;
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
