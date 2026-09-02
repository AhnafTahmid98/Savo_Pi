#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_realsense/camera_health_state.hpp"

namespace
{

constexpr const char * DEFAULT_DEPTH_SIGNAL_TOPIC = "/depth/min_front_m";
constexpr const char * DEFAULT_VO_HEALTH_TOPIC = "/vo/health";
constexpr const char * DEFAULT_OBSTACLE_CLOUD_HEALTH_TOPIC =
  "/savo_perception/obstacle_cloud/health";
constexpr const char * STATUS_TOPIC = "/realsense/status";
constexpr const char * DIAGNOSTICS_TOPIC = "/diagnostics";

class CameraHealthNode : public rclcpp::Node
{
public:
  CameraHealthNode()
  : Node("camera_health_node")
  {
    status_hz_ = std::max(0.1, declare_parameter<double>("status_hz", 2.0));
    stale_timeout_s_ = std::max(
      0.1, declare_parameter<double>("stale_timeout_s", 0.75));

    depth_signal_topic_ = declare_parameter<std::string>(
      "depth_signal_topic", DEFAULT_DEPTH_SIGNAL_TOPIC);
    vo_health_topic_ = declare_parameter<std::string>(
      "vo_health_topic", DEFAULT_VO_HEALTH_TOPIC);
    obstacle_cloud_health_topic_ = declare_parameter<std::string>(
      "obstacle_cloud_health_topic", DEFAULT_OBSTACLE_CLOUD_HEALTH_TOPIC);

    depth_signal_.required = declare_parameter<bool>("require_depth_signal", true);
    const bool legacy_require_aligned_depth =
      declare_parameter<bool>("require_aligned_depth", false);
    vo_health_.required = declare_parameter<bool>(
      "require_vo_health", legacy_require_aligned_depth);
    const bool legacy_require_pointcloud =
      declare_parameter<bool>("require_pointcloud", false);
    obstacle_cloud_.required = declare_parameter<bool>(
      "require_obstacle_cloud_health", legacy_require_pointcloud);

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    if (depth_signal_.required) {
      depth_signal_sub_ = create_subscription<std_msgs::msg::Float32>(
        depth_signal_topic_, reliable_qos,
        [this](const std_msgs::msg::Float32::ConstSharedPtr message) {
          depth_signal_.seen = true;
          depth_signal_.healthy =
            message && std::isfinite(message->data) && message->data > 0.0F;
          depth_signal_.last_update_s = now().seconds();
        });
    }
    if (vo_health_.required) {
      vo_health_sub_ = create_subscription<std_msgs::msg::String>(
        vo_health_topic_, reliable_qos,
        [this](const std_msgs::msg::String::ConstSharedPtr message) {
          vo_health_.seen = true;
          vo_health_.healthy =
            message && savo_realsense::health_text_is_ok(message->data);
          vo_health_.last_update_s = now().seconds();
        });
    }
    if (obstacle_cloud_.required) {
      obstacle_cloud_health_sub_ = create_subscription<std_msgs::msg::Bool>(
        obstacle_cloud_health_topic_, reliable_qos,
        [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
          obstacle_cloud_.seen = true;
          obstacle_cloud_.healthy = message && message->data;
          obstacle_cloud_.last_update_s = now().seconds();
        });
    }

    status_pub_ = create_publisher<std_msgs::msg::String>(STATUS_TOPIC, reliable_qos);
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(DIAGNOSTICS_TOPIC, reliable_qos);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_hz_),
      [this]() {publish_status();});
  }

private:
  diagnostic_msgs::msg::DiagnosticStatus make_signal_diagnostic(
    const std::string & name,
    const std::string & topic,
    const savo_realsense::HealthSignal & signal,
    double now_s) const;

  void publish_status();
  std::string make_status_json(
    const savo_realsense::CameraHealthEvaluation & evaluation) const;

  static const char * json_bool(const bool value)
  {
    return value ? "true" : "false";
  }

  double status_hz_{2.0};
  double stale_timeout_s_{0.75};
  std::string depth_signal_topic_;
  std::string vo_health_topic_;
  std::string obstacle_cloud_health_topic_;
  savo_realsense::HealthSignal depth_signal_;
  savo_realsense::HealthSignal vo_health_;
  savo_realsense::HealthSignal obstacle_cloud_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_signal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vo_health_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_cloud_health_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

diagnostic_msgs::msg::DiagnosticStatus CameraHealthNode::make_signal_diagnostic(
  const std::string & name,
  const std::string & topic,
  const savo_realsense::HealthSignal & signal,
  const double now_s) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "realsense_d435";

  const bool fresh = savo_realsense::signal_is_fresh(
    signal, now_s, stale_timeout_s_);
  if (!signal.required) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "optional";
  } else if (!signal.seen) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "not seen";
  } else if (!fresh) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "stale";
  } else if (!signal.healthy) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "unhealthy";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "healthy";
  }

  auto append_value = [&status](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(entry);
  };
  append_value("topic", topic);
  append_value("required", json_bool(signal.required));
  append_value("seen", json_bool(signal.seen));
  append_value("fresh", json_bool(fresh));
  append_value("healthy", json_bool(signal.healthy));
  append_value(
    "last_age_s",
    signal.seen ? std::to_string(std::max(0.0, now_s - signal.last_update_s)) : "n/a");
  return status;
}

void CameraHealthNode::publish_status()
{
  const auto now_time = now();
  const double now_s = now_time.seconds();
  const auto evaluation = savo_realsense::evaluate_camera_health(
    depth_signal_, vo_health_, obstacle_cloud_, now_s, stale_timeout_s_);

  std_msgs::msg::String status_message;
  status_message.data = make_status_json(evaluation);
  status_pub_->publish(status_message);

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now_time;
  diagnostics.status = {
    make_signal_diagnostic(
      "RealSense depth-front signal", depth_signal_topic_, depth_signal_, now_s),
    make_signal_diagnostic("RealSense VO health", vo_health_topic_, vo_health_, now_s),
    make_signal_diagnostic(
      "RealSense obstacle-cloud health", obstacle_cloud_health_topic_, obstacle_cloud_, now_s),
  };
  diagnostics_pub_->publish(diagnostics);
}

std::string CameraHealthNode::make_status_json(
  const savo_realsense::CameraHealthEvaluation & evaluation) const
{
  std::ostringstream stream;
  stream
    << "{"
    << "\"ok\":" << json_bool(evaluation.ok)
    << ",\"message\":\"" << savo_realsense::camera_health_message(evaluation) << "\""
    << ",\"color_ok\":" << json_bool(evaluation.color_ok)
    << ",\"color_info_ok\":" << json_bool(evaluation.color_info_ok)
    << ",\"depth_ok\":" << json_bool(evaluation.depth_ok)
    << ",\"depth_info_ok\":" << json_bool(evaluation.depth_info_ok)
    << ",\"aligned_depth_ok\":" << json_bool(evaluation.aligned_depth_ok)
    << ",\"require_aligned_depth\":" << json_bool(vo_health_.required)
    << ",\"pointcloud_ok\":" << json_bool(evaluation.pointcloud_ok)
    << ",\"require_pointcloud\":" << json_bool(obstacle_cloud_.required)
    << ",\"depth_signal_ok\":" << json_bool(evaluation.depth_signal_ok)
    << ",\"require_depth_signal\":" << json_bool(depth_signal_.required)
    << ",\"vo_health_ok\":" << json_bool(evaluation.vo_health_ok)
    << ",\"require_vo_health\":" << json_bool(vo_health_.required)
    << ",\"obstacle_cloud_ok\":" << json_bool(evaluation.obstacle_cloud_ok)
    << ",\"require_obstacle_cloud_health\":" << json_bool(obstacle_cloud_.required)
    << "}";
  return stream.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraHealthNode>());
  rclcpp::shutdown();
  return 0;
}
