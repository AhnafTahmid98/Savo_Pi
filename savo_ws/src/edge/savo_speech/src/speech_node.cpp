#include "savo_speech/speech_node.hpp"

#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "savo_speech/constants.hpp"
#include "savo_speech/ros/topic_names.hpp"
#include "savo_speech/version.hpp"

namespace savo_speech
{

namespace
{

[[nodiscard]] std::string owned_string(
  const std::string_view value)
{
  return std::string{value};
}

[[nodiscard]] std::string bool_text(
  const bool value)
{
  return value ? "true" : "false";
}

[[nodiscard]] std::chrono::nanoseconds period_from_rate(
  const double rate_hz)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>{1.0 / rate_hz});
}

[[nodiscard]] rclcpp::QoS state_qos()
{
  rclcpp::QoS qos{rclcpp::KeepLast{1}};

  qos.reliable();
  qos.transient_local();

  return qos;
}

[[nodiscard]] rclcpp::QoS runtime_qos()
{
  rclcpp::QoS qos{rclcpp::KeepLast{10}};

  qos.reliable();
  qos.durability_volatile();

  return qos;
}

[[nodiscard]] diagnostic_msgs::msg::KeyValue make_key_value(
  std::string key,
  std::string value)
{
  diagnostic_msgs::msg::KeyValue item;

  item.key = std::move(key);
  item.value = std::move(value);

  return item;
}

void validate_non_empty(
  const std::string & value,
  const std::string_view parameter_name)
{
  if (value.empty()) {
    throw std::invalid_argument{
            std::string{parameter_name} +
            " must not be empty"};
  }
}

void validate_rate(
  const double value,
  const std::string_view parameter_name)
{
  if (
    !std::isfinite(value) ||
    value <= 0.0 ||
    value > constants::kMaximumTimerRateHz)
  {
    throw std::invalid_argument{
            std::string{parameter_name} +
            " must be finite and in the range (0, 100]"};
  }
}

}  // namespace

SpeechNode::SpeechNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node{
    owned_string(constants::kNodeName),
    options}
{
  try {
    declare_parameters();
    load_parameters();
    validate_parameters();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      get_logger(),
      "Invalid savo_speech configuration: %s",
      exception.what());

    throw;
  }

  readiness_publisher_ =
    create_publisher<std_msgs::msg::String>(
    owned_string(ros::topics::kReadiness),
    state_qos());

  dashboard_publisher_ =
    create_publisher<std_msgs::msg::String>(
    owned_string(ros::topics::kDashboard),
    state_qos());

  heartbeat_publisher_ =
    create_publisher<std_msgs::msg::UInt64>(
    owned_string(ros::topics::kHeartbeat),
    runtime_qos());

  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    owned_string(ros::topics::kDiagnostics),
    runtime_qos());

  configure_initial_state();

  status_timer_ = create_wall_timer(
    period_from_rate(config_.status_publish_rate_hz),
    [this]() {
      publish_runtime_status();
    });

  heartbeat_timer_ = create_wall_timer(
    period_from_rate(config_.heartbeat_rate_hz),
    [this]() {
      publish_heartbeat();
    });

  // Publish immediately. Transient-local readiness and dashboard topics
  // retain the latest value for subscribers that start later.
  publish_runtime_status();

  RCLCPP_INFO(
    get_logger(),
    "savo_speech %s started: profile=%s host_role=%s phase=%s",
    owned_string(version::kVersion).c_str(),
    config_.profile.c_str(),
    config_.host_role.c_str(),
    owned_string(session::to_string(phase_)).c_str());
}

void SpeechNode::declare_parameters()
{
  declare_parameter<bool>(
    "enabled",
    true);

  declare_parameter<std::string>(
    "profile",
    owned_string(constants::kDefaultProfile));

  declare_parameter<std::string>(
    "robot_id",
    owned_string(constants::kDefaultRobotId));

  declare_parameter<std::string>(
    "host_role",
    owned_string(constants::kDefaultHostRole));

  declare_parameter<std::string>(
    "device_id",
    owned_string(constants::kDefaultDeviceId));

  declare_parameter<bool>(
    "audio.required",
    true);

  declare_parameter<std::string>(
    "audio.capture_device",
    owned_string(constants::kDefaultCaptureDevice));

  declare_parameter<std::string>(
    "audio.playback_device",
    owned_string(constants::kDefaultPlaybackDevice));

  declare_parameter<bool>(
    "audio.allow_numeric_device_fallback",
    false);

  declare_parameter<double>(
    "diagnostics.status_publish_rate_hz",
    constants::kDefaultStatusPublishRateHz);

  declare_parameter<double>(
    "diagnostics.heartbeat_rate_hz",
    constants::kDefaultHeartbeatRateHz);
}

void SpeechNode::load_parameters()
{
  config_.enabled =
    get_parameter("enabled").as_bool();

  config_.profile =
    get_parameter("profile").as_string();

  config_.robot_id =
    get_parameter("robot_id").as_string();

  config_.host_role =
    get_parameter("host_role").as_string();

  config_.device_id =
    get_parameter("device_id").as_string();

  config_.audio_required =
    get_parameter("audio.required").as_bool();

  config_.capture_device =
    get_parameter("audio.capture_device").as_string();

  config_.playback_device =
    get_parameter("audio.playback_device").as_string();

  config_.allow_numeric_device_fallback =
    get_parameter(
    "audio.allow_numeric_device_fallback").as_bool();

  config_.status_publish_rate_hz =
    get_parameter(
    "diagnostics.status_publish_rate_hz").as_double();

  config_.heartbeat_rate_hz =
    get_parameter(
    "diagnostics.heartbeat_rate_hz").as_double();
}

void SpeechNode::validate_parameters() const
{
  validate_non_empty(
    config_.profile,
    "profile");

  validate_non_empty(
    config_.robot_id,
    "robot_id");

  validate_non_empty(
    config_.host_role,
    "host_role");

  validate_non_empty(
    config_.device_id,
    "device_id");

  if (config_.host_role != "edge") {
    throw std::invalid_argument{
            "host_role must be 'edge' because savo_speech owns "
            "audio hardware on savo-edge"};
  }

  if (config_.audio_required) {
    validate_non_empty(
      config_.capture_device,
      "audio.capture_device");

    validate_non_empty(
      config_.playback_device,
      "audio.playback_device");
  }

  validate_rate(
    config_.status_publish_rate_hz,
    "diagnostics.status_publish_rate_hz");

  validate_rate(
    config_.heartbeat_rate_hz,
    "diagnostics.heartbeat_rate_hz");
}

void SpeechNode::configure_initial_state()
{
  ready_ = false;
  audio_initialized_ = false;
  savomind_initialized_ = false;

  if (!config_.enabled) {
    phase_ = session::SpeechPhase::Disabled;
    error_ = session::SpeechError::None;
    reason_ = "package_disabled";
    return;
  }

  phase_ = session::SpeechPhase::WaitingForAudio;
  error_ = session::SpeechError::AudioNotInitialized;
  reason_ = "phase_1_audio_backend_not_implemented";
}

void SpeechNode::publish_runtime_status()
{
  std_msgs::msg::String readiness_message;
  readiness_message.data = readiness_text();
  readiness_publisher_->publish(readiness_message);

  std_msgs::msg::String dashboard_message;
  dashboard_message.data = dashboard_text();
  dashboard_publisher_->publish(dashboard_message);

  diagnostics_publisher_->publish(
    create_diagnostics());
}

void SpeechNode::publish_heartbeat()
{
  std_msgs::msg::UInt64 message;

  ++heartbeat_count_;
  message.data = heartbeat_count_;

  heartbeat_publisher_->publish(message);
}

std::string SpeechNode::readiness_text() const
{
  if (!config_.enabled) {
    return "disabled";
  }

  if (ready_) {
    return "ready";
  }

  if (phase_ == session::SpeechPhase::Error) {
    return "error";
  }

  return "waiting_for_audio";
}

std::string SpeechNode::dashboard_text() const
{
  std::ostringstream stream;

  stream
    << "version=" << version::kVersion
    << " profile=" << config_.profile
    << " robot_id=" << config_.robot_id
    << " host_role=" << config_.host_role
    << " device_id=" << config_.device_id
    << " phase=" << session::to_string(phase_)
    << " ready=" << bool_text(ready_)
    << " audio_required=" << bool_text(config_.audio_required)
    << " audio_initialized=" << bool_text(audio_initialized_)
    << " savomind_initialized=" << bool_text(savomind_initialized_)
    << " error=" << session::to_string(error_)
    << " reason=" << reason_;

  return stream.str();
}

diagnostic_msgs::msg::DiagnosticArray
SpeechNode::create_diagnostics() const
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;

  status.name = "savo_speech/runtime";
  status.hardware_id = config_.capture_device;

  if (!config_.enabled) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::OK;

    status.message = "disabled";
  } else if (phase_ == session::SpeechPhase::Error) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    status.message = reason_;
  } else if (!ready_) {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message = reason_;
  } else {
    status.level =
      diagnostic_msgs::msg::DiagnosticStatus::OK;

    status.message = "ready";
  }

  status.values.reserve(16U);

  status.values.push_back(
    make_key_value(
      "package_version",
      owned_string(version::kVersion)));

  status.values.push_back(
    make_key_value(
      "profile",
      config_.profile));

  status.values.push_back(
    make_key_value(
      "robot_id",
      config_.robot_id));

  status.values.push_back(
    make_key_value(
      "host_role",
      config_.host_role));

  status.values.push_back(
    make_key_value(
      "device_id",
      config_.device_id));

  status.values.push_back(
    make_key_value(
      "phase",
      owned_string(session::to_string(phase_))));

  status.values.push_back(
    make_key_value(
      "ready",
      bool_text(ready_)));

  status.values.push_back(
    make_key_value(
      "audio_required",
      bool_text(config_.audio_required)));

  status.values.push_back(
    make_key_value(
      "audio_initialized",
      bool_text(audio_initialized_)));

  status.values.push_back(
    make_key_value(
      "savomind_initialized",
      bool_text(savomind_initialized_)));

  status.values.push_back(
    make_key_value(
      "error",
      owned_string(session::to_string(error_))));

  status.values.push_back(
    make_key_value(
      "reason",
      reason_));

  status.values.push_back(
    make_key_value(
      "capture_device",
      config_.capture_device));

  status.values.push_back(
    make_key_value(
      "playback_device",
      config_.playback_device));

  status.values.push_back(
    make_key_value(
      "numeric_device_fallback",
      bool_text(config_.allow_numeric_device_fallback)));

  status.values.push_back(
    make_key_value(
      "physical_speaker_verification",
      "unsupported_analog_output"));

  message.status.push_back(std::move(status));

  return message;
}

}  // namespace savo_speech
