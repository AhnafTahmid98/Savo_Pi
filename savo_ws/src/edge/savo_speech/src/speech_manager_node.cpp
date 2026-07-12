#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "savo_speech/constants.hpp"
#include "savo_speech/qos_profiles.hpp"
#include "savo_speech/speech_types.hpp"
#include "savo_speech/topic_names.hpp"

using namespace std::chrono_literals;

namespace savo_speech
{

class SpeechManagerNode final : public rclcpp::Node
{
public:
  SpeechManagerNode()
  : Node(std::string(constants::NODE_SPEECH_MANAGER))
  {
    declare_parameters();

    state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(topics::STATE),
      qos::state());

    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(topics::STATUS),
      qos::status());

    health_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      std::string(topics::HEALTH),
      qos::health());

    heartbeat_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(topics::HEARTBEAT),
      qos::heartbeat());

    wake_state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(topics::WAKE_STATE),
      qos::state());

    listening_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      std::string(topics::LISTENING),
      qos::state());

    tts_gate_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      std::string(topics::TTS_GATE),
      qos::state());

    face_state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(topics::FACE_STATE),
      qos::state());

    last_error_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(topics::LAST_ERROR),
      qos::state());

    create_event_publishers();
    create_services();

    startup_timer_ =
      create_wall_timer(
      100ms,
      std::bind(&SpeechManagerNode::complete_startup, this));

    status_timer_ =
      create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(status_publish_period_s_)),
      std::bind(&SpeechManagerNode::publish_status, this));

    heartbeat_timer_ =
      create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(
          heartbeat_publish_period_s_)),
      std::bind(&SpeechManagerNode::publish_heartbeat, this));

    idle_timer_ =
      create_wall_timer(
      200ms,
      std::bind(&SpeechManagerNode::check_idle_timeout, this));

    last_activity_time_ = now();

    publish_state_snapshot();
    publish_status();
    publish_heartbeat();
    publish_health();

    RCLCPP_INFO(
      get_logger(),
      "Dry-run speech manager started: enabled=%s start_awake=%s",
      enabled_ ? "true" : "false",
      start_awake_ ? "true" : "false");
  }

private:
  void declare_parameters()
  {
    enabled_ =
      declare_parameter<bool>("enabled", true);

    start_awake_ =
      declare_parameter<bool>("start_awake", false);

    idle_timeout_s_ =
      declare_parameter<double>(
      "session.idle_timeout_s",
      30.0);

    wake_cooldown_s_ =
      declare_parameter<double>(
      "session.wake_cooldown_s",
      2.0);

    utterance_min_duration_s_ =
      declare_parameter<double>(
      "utterance.min_duration_s",
      0.30);

    utterance_max_duration_s_ =
      declare_parameter<double>(
      "utterance.max_duration_s",
      15.0);

    utterance_speech_start_ms_ =
      declare_parameter<std::int64_t>(
      "utterance.speech_start_ms",
      160);

    utterance_silence_end_ms_ =
      declare_parameter<std::int64_t>(
      "utterance.silence_end_ms",
      700);

    utterance_post_roll_ms_ =
      declare_parameter<std::int64_t>(
      "utterance.post_roll_ms",
      150);

    stt_timeout_s_ =
      declare_parameter<double>(
      "timeout.stt_s",
      20.0);

    thinking_timeout_s_ =
      declare_parameter<double>(
      "timeout.thinking_s",
      30.0);

    tts_timeout_s_ =
      declare_parameter<double>(
      "timeout.tts_s",
      20.0);

    playback_timeout_s_ =
      declare_parameter<double>(
      "timeout.playback_s",
      60.0);

    tts_gate_enabled_ =
      declare_parameter<bool>(
      "tts_gate.enabled",
      true);

    tts_gate_release_delay_ms_ =
      declare_parameter<std::int64_t>(
      "tts_gate.release_delay_ms",
      800);

    barge_in_enabled_ =
      declare_parameter<bool>(
      "barge_in.enabled",
      false);

    status_publish_period_s_ =
      declare_parameter<double>(
      "status.publish_period_s",
      1.0);

    heartbeat_publish_period_s_ =
      declare_parameter<double>(
      "heartbeat.publish_period_s",
      1.0);
  }

  void create_event_publishers()
  {
    for (const auto topic : {
        topics::WAKE_WORD_DETECTED,
        topics::TRANSCRIPT,
        topics::TTS_STARTED,
        topics::TTS_FINISHED,
        topics::TTS_CANCELLED})
    {
      event_publishers_.push_back(
        create_publisher<std_msgs::msg::String>(
          std::string(topic),
          qos::event()));
    }
  }

  void create_services()
  {
    wake_service_ =
      create_service<std_srvs::srv::Trigger>(
      std::string(topics::SERVICE_WAKE),
      std::bind(
        &SpeechManagerNode::handle_wake,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    sleep_service_ =
      create_service<std_srvs::srv::Trigger>(
      std::string(topics::SERVICE_SLEEP),
      std::bind(
        &SpeechManagerNode::handle_sleep,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    start_listening_service_ =
      create_service<std_srvs::srv::Trigger>(
      std::string(topics::SERVICE_START_LISTENING),
      std::bind(
        &SpeechManagerNode::handle_start_listening,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    stop_listening_service_ =
      create_service<std_srvs::srv::Trigger>(
      std::string(topics::SERVICE_STOP_LISTENING),
      std::bind(
        &SpeechManagerNode::handle_stop_listening,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    cancel_service_ =
      create_service<std_srvs::srv::Trigger>(
      std::string(topics::SERVICE_CANCEL),
      std::bind(
        &SpeechManagerNode::handle_cancel,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
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

  static std::string boolean_json(const bool value)
  {
    return value ? "true" : "false";
  }

  static std::string face_state_for(
    const SpeechState state)
  {
    switch (state) {
      case SpeechState::WAKE_DETECTED:
      case SpeechState::LISTENING:
      case SpeechState::UTTERANCE_READY:
        return "listening";

      case SpeechState::TRANSCRIBING:
      case SpeechState::THINKING:
      case SpeechState::SYNTHESIZING:
        return "thinking";

      case SpeechState::SPEAKING:
        return "speaking";

      case SpeechState::ERROR:
        return "error";

      default:
        return "idle";
    }
  }

  void complete_startup()
  {
    startup_timer_->cancel();

    if (!enabled_) {
      state_ = SpeechState::SLEEPING;
      wake_state_ = WakeState::DISABLED;
    } else if (start_awake_) {
      state_ = SpeechState::AWAKE_IDLE;
      wake_state_ = WakeState::AWAKE;
    } else {
      state_ = SpeechState::SLEEPING;
      wake_state_ = WakeState::SLEEPING;
    }

    last_activity_time_ = now();
    publish_all();
  }

  void set_state(
    const SpeechState state,
    const WakeState wake_state)
  {
    state_ = state;
    wake_state_ = wake_state;
    listening_ = state == SpeechState::LISTENING;
    last_activity_time_ = now();

    publish_all();
  }

  void publish_all()
  {
    publish_state_snapshot();
    publish_status();
    publish_health();
  }

  void publish_state_snapshot()
  {
    std_msgs::msg::String state_message;
    state_message.data = std::string(to_string(state_));
    state_publisher_->publish(state_message);

    std_msgs::msg::String wake_message;
    wake_message.data = std::string(to_string(wake_state_));
    wake_state_publisher_->publish(wake_message);

    std_msgs::msg::Bool listening_message;
    listening_message.data = listening_;
    listening_publisher_->publish(listening_message);

    std_msgs::msg::Bool gate_message;
    gate_message.data = tts_gate_active_;
    tts_gate_publisher_->publish(gate_message);

    std_msgs::msg::String face_message;
    face_message.data = face_state_for(state_);
    face_state_publisher_->publish(face_message);

    std_msgs::msg::String error_message;
    error_message.data = last_error_;
    last_error_publisher_->publish(error_message);
  }

  void publish_status()
  {
    const bool ok =
      enabled_ &&
      state_ != SpeechState::ERROR;

    const bool ready_for_voice =
      enabled_ &&
      state_ != SpeechState::STARTING &&
      state_ != SpeechState::ERROR;

    std::ostringstream stream;
    stream
      << "{"
      << "\"available\":" << boolean_json(enabled_) << ","
      << "\"ok\":" << boolean_json(ok) << ","
      << "\"ready_for_voice\":"
      << boolean_json(ready_for_voice) << ","
      << "\"state\":\"" << to_string(state_) << "\","
      << "\"wake_state\":\""
      << to_string(wake_state_) << "\","
      << "\"listening\":"
      << boolean_json(listening_) << ","
      << "\"tts_gate_active\":"
      << boolean_json(tts_gate_active_) << ","
      << "\"face_state\":\""
      << face_state_for(state_) << "\","
      << "\"last_error\":";

    if (last_error_.empty()) {
      stream << "null";
    } else {
      stream << "\"" << last_error_ << "\"";
    }

    stream << "}";

    std_msgs::msg::String message;
    message.data = stream.str();
    status_publisher_->publish(message);
  }

  void publish_heartbeat()
  {
    ++heartbeat_sequence_;

    std::ostringstream stream;
    stream
      << std::fixed
      << std::setprecision(3)
      << "{"
      << "\"node\":\"speech_manager_node\","
      << "\"seq\":" << heartbeat_sequence_ << ","
      << "\"stamp_s\":" << now().seconds() << ","
      << "\"state\":\"" << to_string(state_) << "\""
      << "}";

    std_msgs::msg::String message;
    message.data = stream.str();
    heartbeat_publisher_->publish(message);
  }

  void publish_health()
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "savo_speech/speech_manager";
    status.hardware_id = "speech-manager-dryrun";

    if (!enabled_) {
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "speech manager disabled";
    } else if (state_ == SpeechState::ERROR) {
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = last_error_;
    } else {
      status.level =
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "dry-run speech manager ready";
    }

    status.values.push_back(
      make_key_value(
        "state",
        std::string(to_string(state_))));

    status.values.push_back(
      make_key_value(
        "wake_state",
        std::string(to_string(wake_state_))));

    status.values.push_back(
      make_key_value(
        "listening",
        listening_ ? "true" : "false"));

    status.values.push_back(
      make_key_value(
        "tts_gate_active",
        tts_gate_active_ ? "true" : "false"));

    status.values.push_back(
      make_key_value(
        "idle_timeout_s",
        std::to_string(idle_timeout_s_)));

    status.values.push_back(
      make_key_value(
        "tts_gate_release_delay_ms",
        std::to_string(tts_gate_release_delay_ms_)));

    message.status.push_back(std::move(status));
    health_publisher_->publish(message);
  }

  bool manager_available(
    std_srvs::srv::Trigger::Response & response) const
  {
    if (enabled_) {
      return true;
    }

    response.success = false;
    response.message = "speech manager is disabled";
    return false;
  }

  void handle_wake(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!manager_available(*response)) {
      return;
    }

    set_state(
      SpeechState::AWAKE_IDLE,
      WakeState::AWAKE);

    response->success = true;
    response->message = "speech subsystem awake";
  }

  void handle_sleep(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!manager_available(*response)) {
      return;
    }

    tts_gate_active_ = false;

    set_state(
      SpeechState::SLEEPING,
      WakeState::SLEEPING);

    response->success = true;
    response->message = "speech subsystem sleeping";
  }

  void handle_start_listening(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!manager_available(*response)) {
      return;
    }

    if (state_ == SpeechState::LISTENING) {
      response->success = true;
      response->message = "already listening";
      return;
    }

    set_state(
      SpeechState::LISTENING,
      WakeState::AWAKE);

    response->success = true;
    response->message = "listening started";
  }

  void handle_stop_listening(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!manager_available(*response)) {
      return;
    }

    if (state_ != SpeechState::LISTENING) {
      response->success = false;
      response->message = "speech subsystem is not listening";
      return;
    }

    set_state(
      SpeechState::AWAKE_IDLE,
      WakeState::AWAKE);

    response->success = true;
    response->message = "listening stopped";
  }

  void handle_cancel(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!manager_available(*response)) {
      return;
    }

    tts_gate_active_ = false;

    if (state_ == SpeechState::SLEEPING) {
      publish_all();

      response->success = true;
      response->message = "nothing active to cancel";
      return;
    }

    set_state(
      SpeechState::AWAKE_IDLE,
      WakeState::AWAKE);

    response->success = true;
    response->message = "active speech turn cancelled";
  }

  void check_idle_timeout()
  {
    if (
      !enabled_ ||
      idle_timeout_s_ <= 0.0 ||
      state_ != SpeechState::AWAKE_IDLE)
    {
      return;
    }

    const double idle_age_s =
      (now() - last_activity_time_).seconds();

    if (idle_age_s < idle_timeout_s_) {
      return;
    }

    set_state(
      SpeechState::SLEEPING,
      WakeState::SLEEPING);

    RCLCPP_INFO(
      get_logger(),
      "Speech session returned to sleep after %.2f seconds",
      idle_age_s);
  }

  bool enabled_{true};
  bool start_awake_{false};

  double idle_timeout_s_{30.0};
  double wake_cooldown_s_{2.0};

  double utterance_min_duration_s_{0.30};
  double utterance_max_duration_s_{15.0};
  std::int64_t utterance_speech_start_ms_{160};
  std::int64_t utterance_silence_end_ms_{700};
  std::int64_t utterance_post_roll_ms_{150};

  double stt_timeout_s_{20.0};
  double thinking_timeout_s_{30.0};
  double tts_timeout_s_{20.0};
  double playback_timeout_s_{60.0};

  bool tts_gate_enabled_{true};
  std::int64_t tts_gate_release_delay_ms_{800};
  bool barge_in_enabled_{false};

  double status_publish_period_s_{1.0};
  double heartbeat_publish_period_s_{1.0};

  SpeechState state_{SpeechState::STARTING};
  WakeState wake_state_{WakeState::DISABLED};

  bool listening_{false};
  bool tts_gate_active_{false};
  std::string last_error_;

  std::uint64_t heartbeat_sequence_{0};
  rclcpp::Time last_activity_time_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    status_publisher_;

  rclcpp::Publisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    health_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    heartbeat_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    wake_state_publisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    listening_publisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    tts_gate_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    face_state_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    last_error_publisher_;

  std::vector<
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>
  event_publishers_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    wake_service_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    sleep_service_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    start_listening_service_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    stop_listening_service_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    cancel_service_;

  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr idle_timer_;
};

}  // namespace savo_speech

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<savo_speech::SpeechManagerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("speech_manager_node"),
      "Fatal startup error: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
