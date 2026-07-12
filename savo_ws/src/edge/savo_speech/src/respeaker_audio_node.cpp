#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "savo_speech/constants.hpp"
#include "savo_speech/qos_profiles.hpp"
#include "savo_speech/topic_names.hpp"

using namespace std::chrono_literals;

namespace savo_speech
{

class RespeakerAudioNode final : public rclcpp::Node
{
public:
  RespeakerAudioNode()
  : Node(std::string(constants::NODE_RESPEAKER_AUDIO))
  {
    declare_parameters();
    validate_parameters();

    input_muted_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      std::string(topics::INPUT_MUTED),
      qos::state());

    output_muted_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      std::string(topics::OUTPUT_MUTED),
      qos::state());

    health_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      std::string(topics::HEALTH),
      qos::health());

    mute_input_service_ =
      create_service<std_srvs::srv::SetBool>(
      std::string(topics::SERVICE_MUTE_INPUT),
      std::bind(
        &RespeakerAudioNode::handle_mute_input,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    mute_output_service_ =
      create_service<std_srvs::srv::SetBool>(
      std::string(topics::SERVICE_MUTE_OUTPUT),
      std::bind(
        &RespeakerAudioNode::handle_mute_output,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    reload_audio_service_ =
      create_service<std_srvs::srv::Trigger>(
      std::string(topics::SERVICE_RELOAD_AUDIO_DEVICE),
      std::bind(
        &RespeakerAudioNode::handle_reload_audio_device,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    health_timer_ =
      create_wall_timer(
      1s,
      std::bind(&RespeakerAudioNode::publish_health, this));

    publish_mute_states();
    publish_health();

    RCLCPP_INFO(
      get_logger(),
      "Dry-run audio runtime ready: enabled=%s backend=%s "
      "sample_rate=%ld Hz frame=%ld ms",
      enabled_ ? "true" : "false",
      backend_.c_str(),
      static_cast<long>(sample_rate_hz_),
      static_cast<long>(frame_ms_));
  }

private:
  void declare_parameters()
  {
    enabled_ = declare_parameter<bool>("enabled", false);
    backend_ = declare_parameter<std::string>("backend", "dryrun");

    sample_rate_hz_ =
      declare_parameter<std::int64_t>("sample_rate_hz", 16000);

    sample_format_ =
      declare_parameter<std::string>("sample_format", "S16_LE");

    frame_ms_ =
      declare_parameter<std::int64_t>("frame_ms", 20);

    capture_device_hint_ =
      declare_parameter<std::string>(
      "capture.device_hint",
      "ReSpeaker");

    capture_alsa_device_ =
      declare_parameter<std::string>(
      "capture.alsa_device",
      "");

    capture_device_channels_ =
      declare_parameter<std::int64_t>(
      "capture.device_channels",
      0);

    capture_processed_channel_ =
      declare_parameter<std::int64_t>(
      "capture.processed_channel",
      0);

    capture_output_channels_ =
      declare_parameter<std::int64_t>(
      "capture.output_channels",
      1);

    playback_device_hint_ =
      declare_parameter<std::string>(
      "playback.device_hint",
      "ReSpeaker");

    playback_alsa_device_ =
      declare_parameter<std::string>(
      "playback.alsa_device",
      "");

    playback_channels_ =
      declare_parameter<std::int64_t>(
      "playback.channels",
      2);

    ring_buffer_ms_ =
      declare_parameter<std::int64_t>(
      "buffering.ring_buffer_ms",
      5000);

    pre_roll_ms_ =
      declare_parameter<std::int64_t>(
      "buffering.pre_roll_ms",
      400);

    reconnect_enabled_ =
      declare_parameter<bool>(
      "reconnect.enabled",
      true);

    reconnect_interval_s_ =
      declare_parameter<double>(
      "reconnect.interval_s",
      2.0);

    reconnect_max_attempts_ =
      declare_parameter<std::int64_t>(
      "reconnect.max_attempts",
      0);

    dryrun_capture_enabled_ =
      declare_parameter<bool>(
      "dryrun.capture_enabled",
      true);

    dryrun_playback_enabled_ =
      declare_parameter<bool>(
      "dryrun.playback_enabled",
      true);
  }

  void validate_parameters() const
  {
    if (backend_ != "dryrun" && backend_ != "alsa") {
      throw std::invalid_argument(
              "backend must be 'dryrun' or 'alsa'");
    }

    if (enabled_ && backend_ != "dryrun") {
      throw std::runtime_error(
              "Phase 0D supports only the dryrun audio backend");
    }

    if (sample_rate_hz_ < 8000 || sample_rate_hz_ > 48000) {
      throw std::invalid_argument(
              "sample_rate_hz must be between 8000 and 48000");
    }

    if (sample_format_ != "S16_LE") {
      throw std::invalid_argument(
              "Phase 0D supports only S16_LE");
    }

    if (
      frame_ms_ != 10 &&
      frame_ms_ != 20 &&
      frame_ms_ != 30)
    {
      throw std::invalid_argument(
              "frame_ms must be 10, 20, or 30");
    }

    if (
      capture_device_channels_ > 0 &&
      capture_processed_channel_ >= capture_device_channels_)
    {
      throw std::invalid_argument(
              "capture.processed_channel must be below "
              "capture.device_channels");
    }

    if (pre_roll_ms_ >= ring_buffer_ms_) {
      throw std::invalid_argument(
              "buffering.pre_roll_ms must be below "
              "buffering.ring_buffer_ms");
    }
  }

  static diagnostic_msgs::msg::KeyValue make_key_value(
    std::string key,
    std::string value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = std::move(key);
    item.value = std::move(value);
    return item;
  }

  void publish_mute_states()
  {
    std_msgs::msg::Bool input_message;
    input_message.data = input_muted_;
    input_muted_publisher_->publish(input_message);

    std_msgs::msg::Bool output_message;
    output_message.data = output_muted_;
    output_muted_publisher_->publish(output_message);
  }

  void publish_health()
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "savo_speech/respeaker_audio";
    status.hardware_id = "respeaker-dryrun";

    if (!enabled_) {
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "audio runtime disabled";
    } else {
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "dry-run audio runtime ready";
    }

    status.values.push_back(
      make_key_value(
        "enabled",
        enabled_ ? "true" : "false"));

    status.values.push_back(
      make_key_value(
        "backend",
        backend_));

    status.values.push_back(
      make_key_value(
        "sample_rate_hz",
        std::to_string(sample_rate_hz_)));

    status.values.push_back(
      make_key_value(
        "frame_ms",
        std::to_string(frame_ms_)));

    status.values.push_back(
      make_key_value(
        "input_muted",
        input_muted_ ? "true" : "false"));

    status.values.push_back(
      make_key_value(
        "output_muted",
        output_muted_ ? "true" : "false"));

    status.values.push_back(
      make_key_value(
        "reload_generation",
        std::to_string(reload_generation_)));

    message.status.push_back(std::move(status));
    health_publisher_->publish(message);
  }

  void handle_mute_input(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (!enabled_) {
      response->success = false;
      response->message = "audio runtime is disabled";
      return;
    }

    input_muted_ = request->data;
    publish_mute_states();
    publish_health();

    response->success = true;
    response->message =
      input_muted_ ? "input muted" : "input unmuted";
  }

  void handle_mute_output(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (!enabled_) {
      response->success = false;
      response->message = "audio runtime is disabled";
      return;
    }

    output_muted_ = request->data;
    publish_mute_states();
    publish_health();

    response->success = true;
    response->message =
      output_muted_ ? "output muted" : "output unmuted";
  }

  void handle_reload_audio_device(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!enabled_) {
      response->success = false;
      response->message = "audio runtime is disabled";
      return;
    }

    ++reload_generation_;
    publish_health();

    response->success = true;
    response->message =
      "dry-run audio device reloaded";
  }

  bool enabled_{false};
  std::string backend_{"dryrun"};

  std::int64_t sample_rate_hz_{16000};
  std::string sample_format_{"S16_LE"};
  std::int64_t frame_ms_{20};

  std::string capture_device_hint_;
  std::string capture_alsa_device_;
  std::int64_t capture_device_channels_{0};
  std::int64_t capture_processed_channel_{0};
  std::int64_t capture_output_channels_{1};

  std::string playback_device_hint_;
  std::string playback_alsa_device_;
  std::int64_t playback_channels_{2};

  std::int64_t ring_buffer_ms_{5000};
  std::int64_t pre_roll_ms_{400};

  bool reconnect_enabled_{true};
  double reconnect_interval_s_{2.0};
  std::int64_t reconnect_max_attempts_{0};

  bool dryrun_capture_enabled_{true};
  bool dryrun_playback_enabled_{true};

  bool input_muted_{false};
  bool output_muted_{false};
  std::uint64_t reload_generation_{0};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    input_muted_publisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    output_muted_publisher_;

  rclcpp::Publisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    health_publisher_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    mute_input_service_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    mute_output_service_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    reload_audio_service_;

  rclcpp::TimerBase::SharedPtr health_timer_;
};

}  // namespace savo_speech

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<savo_speech::RespeakerAudioNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("respeaker_audio_node"),
      "Fatal startup error: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
