// Copyright 2026 Ahnaf Tahmid
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "savo_speech/speech_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "savo_speech/audio/audio_format.hpp"
#include "savo_speech/constants.hpp"
#include "savo_speech/ros/topic_names.hpp"
#include "savo_speech/version.hpp"
#include "savo_speech/wake_word/wake_word_asset_resolver.hpp"

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

void validate_probability(
  const double value,
  const std::string_view parameter_name)
{
  if (
    !std::isfinite(value) ||
    value < 0.0 ||
    value > 1.0)
  {
    throw std::invalid_argument{
            std::string{parameter_name} +
            " must be finite and in the range [0, 1]"};
  }
}

void validate_integer_range(
  const std::int64_t value,
  const std::int64_t minimum,
  const std::int64_t maximum,
  const std::string_view parameter_name)
{
  if (value < minimum || value > maximum) {
    throw std::invalid_argument{
            std::string{parameter_name} +
            " must be in the range [" +
            std::to_string(minimum) +
            ", " +
            std::to_string(maximum) +
            "]"};
  }
}

[[nodiscard]] std::size_t pre_roll_samples(
  const std::int64_t sample_rate_hz,
  const std::int64_t pre_roll_ms)
{
  const auto sample_rate =
    static_cast<std::uint64_t>(sample_rate_hz);

  const auto duration_ms =
    static_cast<std::uint64_t>(pre_roll_ms);

  if (
    sample_rate >
    std::numeric_limits<std::uint64_t>::max() /
    duration_ms)
  {
    throw std::overflow_error{
            "audio pre-roll sample calculation overflowed"};
  }

  const std::uint64_t samples =
    sample_rate * duration_ms / 1000U;

  if (
    samples == 0U ||
    samples >
    static_cast<std::uint64_t>(
      std::numeric_limits<std::size_t>::max()))
  {
    throw std::invalid_argument{
            "audio pre-roll resolved to an invalid sample count"};
  }

  return static_cast<std::size_t>(samples);
}

[[nodiscard]] session::SpeechError classify_audio_error(
  const std::string & message)
{
  if (message.find("capture") != std::string::npos) {
    return session::SpeechError::CaptureStreamFailed;
  }

  if (message.find("playback") != std::string::npos) {
    return session::SpeechError::PlaybackStreamFailed;
  }

  return session::SpeechError::AudioNotInitialized;
}

[[nodiscard]] std::uint8_t diagnostic_level(
  const audio::AudioRuntimeState state)
{
  switch (state) {
    case audio::AudioRuntimeState::Running:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;

    case audio::AudioRuntimeState::Starting:
    case audio::AudioRuntimeState::Stopping:
    case audio::AudioRuntimeState::Stopped:
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;

    case audio::AudioRuntimeState::Faulted:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
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

  publish_runtime_status();

  RCLCPP_INFO(
    get_logger(),
    "savo_speech %s started: profile=%s host_role=%s "
    "phase=%s audio_initialized=%s reason=%s",
    owned_string(version::kVersion).c_str(),
    config_.profile.c_str(),
    config_.host_role.c_str(),
    owned_string(session::to_string(phase_)).c_str(),
    bool_text(audio_initialized_).c_str(),
    reason_.c_str());
}

SpeechNode::~SpeechNode()
{
  phase_ = session::SpeechPhase::ShuttingDown;
  ready_ = false;

  shutdown_audio_runtime();
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

  declare_parameter<bool>(
    "audio.require_exact_sample_rate",
    true);

  declare_parameter<std::int64_t>(
    "audio.sample_rate_hz",
    16000);

  declare_parameter<std::int64_t>(
    "audio.capture_channels",
    6);

  declare_parameter<std::int64_t>(
    "audio.playback_channels",
    1);

  declare_parameter<std::int64_t>(
    "audio.selected_channel",
    0);

  declare_parameter<std::int64_t>(
    "audio.period_frames",
    320);

  declare_parameter<std::int64_t>(
    "audio.periods",
    4);

  declare_parameter<std::int64_t>(
    "audio.chunk_frames",
    320);

  declare_parameter<std::int64_t>(
    "audio.pre_roll_ms",
    1000);

  declare_parameter<std::int64_t>(
    "audio.post_playback_hold_ms",
    250);

  declare_parameter<std::int64_t>(
    "audio.capture_queue_capacity_frames",
    100);

  declare_parameter<std::int64_t>(
    "audio.playback_queue_capacity",
    8);

  declare_parameter<std::int64_t>(
    "audio.processing.wait_timeout_ms",
    100);

  declare_parameter<std::int64_t>(
    "audio.processing.freshness_timeout_ms",
    1000);

  declare_parameter<bool>(
    "audio.processing.fault_on_processor_error",
    true);

  declare_parameter<bool>(
    "wake_word.enabled",
    false);

  declare_parameter<bool>(
    "wake_word.required",
    true);

  declare_parameter<std::string>(
    "wake_word.profile",
    "default");

  declare_parameter<std::string>(
    "wake_word.acoustic_model_path",
    "/usr/share/pocketsphinx/model/en-us/en-us");

  declare_parameter<std::string>(
    "wake_word.dictionary_path",
    "");

  declare_parameter<std::string>(
    "wake_word.keyword_file_path",
    "");

  declare_parameter<std::string>(
    "wake_word.search_name",
    "savo_wake_words");

  declare_parameter<bool>(
    "wake_word.suppress_decoder_log",
    true);

  declare_parameter<double>(
    "wake_word.confidence_threshold",
    0.65);

  declare_parameter<std::int64_t>(
    "wake_word.required_consecutive_detections",
    1);

  declare_parameter<std::int64_t>(
    "wake_word.cooldown_ms",
    2000);

  declare_parameter<std::int64_t>(
    "wake_word.event_queue_capacity",
    8);

  declare_parameter<bool>(
    "vad.enabled",
    false);

  declare_parameter<bool>(
    "vad.required",
    true);

  declare_parameter<std::int64_t>(
    "vad.backend.startup_calibration_frames",
    25);

  declare_parameter<double>(
    "vad.backend.initial_noise_floor_rms",
    0.005);

  declare_parameter<double>(
    "vad.backend.minimum_noise_floor_rms",
    0.0005);

  declare_parameter<double>(
    "vad.backend.maximum_noise_floor_rms",
    0.250);

  declare_parameter<double>(
    "vad.backend.noise_floor_update_alpha",
    0.05);

  declare_parameter<double>(
    "vad.backend.minimum_speech_rms",
    0.010);

  declare_parameter<double>(
    "vad.backend.speech_onset_snr_db",
    8.0);

  declare_parameter<double>(
    "vad.backend.speech_saturation_snr_db",
    20.0);

  declare_parameter<std::int64_t>(
    "vad.backend.clipping_threshold",
    32760);

  declare_parameter<double>(
    "vad.processor.speech_start_threshold",
    0.65);

  declare_parameter<double>(
    "vad.processor.speech_end_threshold",
    0.35);

  declare_parameter<std::int64_t>(
    "vad.processor.required_start_frames",
    3);

  declare_parameter<std::int64_t>(
    "vad.processor.required_end_frames",
    10);

  declare_parameter<std::int64_t>(
    "vad.processor.event_queue_capacity",
    8);

  declare_parameter<bool>(
    "utterance_session.enabled",
    false);

  declare_parameter<bool>(
    "utterance_session.required",
    true);

  declare_parameter<std::int64_t>(
    "utterance_session.pre_roll_ms",
    1000);

  declare_parameter<std::int64_t>(
    "utterance_session.speech_start_timeout_ms",
    3000);

  declare_parameter<std::int64_t>(
    "utterance_session.maximum_duration_ms",
    15000);

  declare_parameter<std::int64_t>(
    "utterance_session.completed_queue_capacity",
    4);

  declare_parameter<bool>(
    "utterance_serialization.enabled",
    false);

  declare_parameter<bool>(
    "utterance_serialization.required",
    true);

  declare_parameter<std::int64_t>(
    "utterance_serialization.output_queue_capacity",
    4);

  declare_parameter<std::int64_t>(
    "utterance_serialization.source_wait_timeout_ms",
    100);

  declare_parameter<std::int64_t>(
    "utterance_serialization.maximum_wav_bytes",
    2 * 1024 * 1024);

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

  config_.require_exact_sample_rate =
    get_parameter(
    "audio.require_exact_sample_rate").as_bool();

  config_.sample_rate_hz =
    get_parameter("audio.sample_rate_hz").as_int();

  config_.capture_channels =
    get_parameter("audio.capture_channels").as_int();

  config_.playback_channels =
    get_parameter("audio.playback_channels").as_int();

  config_.selected_channel =
    get_parameter("audio.selected_channel").as_int();

  config_.period_frames =
    get_parameter("audio.period_frames").as_int();

  config_.periods =
    get_parameter("audio.periods").as_int();

  config_.chunk_frames =
    get_parameter("audio.chunk_frames").as_int();

  config_.pre_roll_ms =
    get_parameter("audio.pre_roll_ms").as_int();

  config_.post_playback_hold_ms =
    get_parameter(
    "audio.post_playback_hold_ms").as_int();

  config_.capture_queue_capacity_frames =
    get_parameter(
    "audio.capture_queue_capacity_frames").as_int();

  config_.playback_queue_capacity =
    get_parameter(
    "audio.playback_queue_capacity").as_int();

  config_.processing_wait_timeout_ms =
    get_parameter(
    "audio.processing.wait_timeout_ms").as_int();

  config_.processing_freshness_timeout_ms =
    get_parameter(
    "audio.processing.freshness_timeout_ms").as_int();

  config_.processing_fault_on_processor_error =
    get_parameter(
    "audio.processing.fault_on_processor_error").as_bool();

  config_.wake_word_enabled =
    get_parameter("wake_word.enabled").as_bool();

  config_.wake_word_required =
    get_parameter("wake_word.required").as_bool();

  config_.wake_word_profile =
    get_parameter("wake_word.profile").as_string();

  config_.wake_word_acoustic_model_path =
    get_parameter(
    "wake_word.acoustic_model_path").as_string();

  config_.wake_word_dictionary_path =
    get_parameter(
    "wake_word.dictionary_path").as_string();

  config_.wake_word_keyword_file_path =
    get_parameter(
    "wake_word.keyword_file_path").as_string();

  config_.wake_word_search_name =
    get_parameter(
    "wake_word.search_name").as_string();

  config_.wake_word_suppress_decoder_log =
    get_parameter(
    "wake_word.suppress_decoder_log").as_bool();

  config_.wake_word_confidence_threshold =
    get_parameter(
    "wake_word.confidence_threshold").as_double();

  config_.wake_word_consecutive_detections =
    get_parameter(
    "wake_word.required_consecutive_detections").as_int();

  config_.wake_word_cooldown_ms =
    get_parameter("wake_word.cooldown_ms").as_int();

  config_.wake_word_event_queue_capacity =
    get_parameter(
    "wake_word.event_queue_capacity").as_int();

  config_.vad_enabled =
    get_parameter("vad.enabled").as_bool();

  config_.vad_required =
    get_parameter("vad.required").as_bool();

  config_.vad_startup_calibration_frames =
    get_parameter(
    "vad.backend.startup_calibration_frames").as_int();

  config_.vad_initial_noise_floor_rms =
    get_parameter(
    "vad.backend.initial_noise_floor_rms").as_double();

  config_.vad_minimum_noise_floor_rms =
    get_parameter(
    "vad.backend.minimum_noise_floor_rms").as_double();

  config_.vad_maximum_noise_floor_rms =
    get_parameter(
    "vad.backend.maximum_noise_floor_rms").as_double();

  config_.vad_noise_floor_update_alpha =
    get_parameter(
    "vad.backend.noise_floor_update_alpha").as_double();

  config_.vad_minimum_speech_rms =
    get_parameter(
    "vad.backend.minimum_speech_rms").as_double();

  config_.vad_speech_onset_snr_db =
    get_parameter(
    "vad.backend.speech_onset_snr_db").as_double();

  config_.vad_speech_saturation_snr_db =
    get_parameter(
    "vad.backend.speech_saturation_snr_db").as_double();

  config_.vad_clipping_threshold =
    get_parameter(
    "vad.backend.clipping_threshold").as_int();

  config_.vad_speech_start_threshold =
    get_parameter(
    "vad.processor.speech_start_threshold").as_double();

  config_.vad_speech_end_threshold =
    get_parameter(
    "vad.processor.speech_end_threshold").as_double();

  config_.vad_required_start_frames =
    get_parameter(
    "vad.processor.required_start_frames").as_int();

  config_.vad_required_end_frames =
    get_parameter(
    "vad.processor.required_end_frames").as_int();

  config_.vad_event_queue_capacity =
    get_parameter(
    "vad.processor.event_queue_capacity").as_int();

  config_.utterance_session_enabled =
    get_parameter(
    "utterance_session.enabled").as_bool();

  config_.utterance_session_required =
    get_parameter(
    "utterance_session.required").as_bool();

  config_.utterance_session_pre_roll_ms =
    get_parameter(
    "utterance_session.pre_roll_ms").as_int();

  config_.utterance_session_speech_start_timeout_ms =
    get_parameter(
    "utterance_session.speech_start_timeout_ms").as_int();

  config_.utterance_session_maximum_duration_ms =
    get_parameter(
    "utterance_session.maximum_duration_ms").as_int();

  config_.utterance_session_completed_queue_capacity =
    get_parameter(
    "utterance_session.completed_queue_capacity").as_int();

  config_.utterance_serialization_enabled =
    get_parameter(
    "utterance_serialization.enabled").as_bool();

  config_.utterance_serialization_required =
    get_parameter(
    "utterance_serialization.required").as_bool();

  config_.
  utterance_serialization_output_queue_capacity =
    get_parameter(
    "utterance_serialization.output_queue_capacity").
    as_int();

  config_.
  utterance_serialization_source_wait_timeout_ms =
    get_parameter(
    "utterance_serialization.source_wait_timeout_ms").
    as_int();

  config_.utterance_serialization_maximum_wav_bytes =
    get_parameter(
    "utterance_serialization.maximum_wav_bytes").
    as_int();

  config_.status_publish_rate_hz =
    get_parameter(
    "diagnostics.status_publish_rate_hz").as_double();

  config_.heartbeat_rate_hz =
    get_parameter(
    "diagnostics.heartbeat_rate_hz").as_double();
}

void SpeechNode::validate_parameters() const
{
  validate_non_empty(config_.profile, "profile");
  validate_non_empty(config_.robot_id, "robot_id");
  validate_non_empty(config_.host_role, "host_role");
  validate_non_empty(config_.device_id, "device_id");

  if (config_.host_role != "edge") {
    throw std::invalid_argument{
            "host_role must be 'edge' because savo_speech owns "
            "audio hardware on savo-edge"};
  }

  if (config_.allow_numeric_device_fallback) {
    throw std::invalid_argument{
            "audio.allow_numeric_device_fallback is not supported; "
            "configure stable ALSA PCM names instead"};
  }

  if (config_.audio_required) {
    validate_non_empty(
      config_.capture_device,
      "audio.capture_device");

    validate_non_empty(
      config_.playback_device,
      "audio.playback_device");
  }

  validate_integer_range(
    config_.sample_rate_hz,
    8000,
    192000,
    "audio.sample_rate_hz");

  validate_integer_range(
    config_.capture_channels,
    1,
    32,
    "audio.capture_channels");

  validate_integer_range(
    config_.playback_channels,
    1,
    32,
    "audio.playback_channels");

  validate_integer_range(
    config_.selected_channel,
    0,
    config_.capture_channels - 1,
    "audio.selected_channel");

  validate_integer_range(
    config_.period_frames,
    1,
    65536,
    "audio.period_frames");

  validate_integer_range(
    config_.periods,
    2,
    32,
    "audio.periods");

  validate_integer_range(
    config_.chunk_frames,
    1,
    65536,
    "audio.chunk_frames");

  validate_integer_range(
    config_.pre_roll_ms,
    1,
    10000,
    "audio.pre_roll_ms");

  validate_integer_range(
    config_.post_playback_hold_ms,
    0,
    10000,
    "audio.post_playback_hold_ms");

  validate_integer_range(
    config_.capture_queue_capacity_frames,
    1,
    10000,
    "audio.capture_queue_capacity_frames");

  validate_integer_range(
    config_.playback_queue_capacity,
    1,
    1024,
    "audio.playback_queue_capacity");

  validate_integer_range(
    config_.processing_wait_timeout_ms,
    1,
    5000,
    "audio.processing.wait_timeout_ms");

  validate_integer_range(
    config_.processing_freshness_timeout_ms,
    config_.processing_wait_timeout_ms,
    30000,
    "audio.processing.freshness_timeout_ms");

  if (config_.wake_word_enabled) {
    if (!config_.audio_required) {
      throw std::invalid_argument{
              "wake_word.enabled requires audio.required=true"};
    }

    if (config_.sample_rate_hz != 16000) {
      throw std::invalid_argument{
              "PocketSphinx wake-word processing requires "
              "audio.sample_rate_hz=16000"};
    }

    validate_non_empty(
      config_.wake_word_profile,
      "wake_word.profile");

    if (
      !wake_word::is_supported_wake_word_profile(
        config_.wake_word_profile))
    {
      throw std::invalid_argument{
              "wake_word.profile must be default, "
              "extended, or custom"};
    }

    validate_non_empty(
      config_.wake_word_acoustic_model_path,
      "wake_word.acoustic_model_path");

    validate_non_empty(
      config_.wake_word_search_name,
      "wake_word.search_name");

    if (
      config_.wake_word_profile == "custom" &&
      config_.wake_word_keyword_file_path.empty())
    {
      throw std::invalid_argument{
              "custom wake_word.profile requires "
              "wake_word.keyword_file_path"};
    }

    validate_probability(
      config_.wake_word_confidence_threshold,
      "wake_word.confidence_threshold");

    validate_integer_range(
      config_.wake_word_consecutive_detections,
      1,
      10,
      "wake_word.required_consecutive_detections");

    validate_integer_range(
      config_.wake_word_cooldown_ms,
      0,
      60000,
      "wake_word.cooldown_ms");

    validate_integer_range(
      config_.wake_word_event_queue_capacity,
      1,
      1024,
      "wake_word.event_queue_capacity");
  }

  if (config_.vad_enabled) {
    if (!config_.audio_required) {
      throw std::invalid_argument{
              "vad.enabled requires audio.required=true"};
    }

    validate_integer_range(
      config_.vad_startup_calibration_frames,
      0,
      100000,
      "vad.backend.startup_calibration_frames");

    validate_integer_range(
      config_.vad_clipping_threshold,
      1,
      32768,
      "vad.backend.clipping_threshold");

    validate_integer_range(
      config_.vad_required_start_frames,
      1,
      1000,
      "vad.processor.required_start_frames");

    validate_integer_range(
      config_.vad_required_end_frames,
      1,
      1000,
      "vad.processor.required_end_frames");

    validate_integer_range(
      config_.vad_event_queue_capacity,
      1,
      1024,
      "vad.processor.event_queue_capacity");

    vad::AdaptiveEnergyVadConfig backend_config;

    backend_config.expected_sample_rate_hz =
      static_cast<std::uint32_t>(
      config_.sample_rate_hz);

    backend_config.startup_calibration_frames =
      static_cast<std::size_t>(
      config_.vad_startup_calibration_frames);

    backend_config.initial_noise_floor_rms =
      config_.vad_initial_noise_floor_rms;

    backend_config.minimum_noise_floor_rms =
      config_.vad_minimum_noise_floor_rms;

    backend_config.maximum_noise_floor_rms =
      config_.vad_maximum_noise_floor_rms;

    backend_config.noise_floor_update_alpha =
      config_.vad_noise_floor_update_alpha;

    backend_config.minimum_speech_rms =
      config_.vad_minimum_speech_rms;

    backend_config.speech_onset_snr_db =
      config_.vad_speech_onset_snr_db;

    backend_config.speech_saturation_snr_db =
      config_.vad_speech_saturation_snr_db;

    backend_config.clipping_threshold =
      static_cast<std::uint32_t>(
      config_.vad_clipping_threshold);

    if (!backend_config.is_valid()) {
      throw std::invalid_argument{
              "invalid vad.backend parameter combination"};
    }

    vad::VadProcessorConfig processor_config;

    processor_config.speech_start_threshold =
      config_.vad_speech_start_threshold;

    processor_config.speech_end_threshold =
      config_.vad_speech_end_threshold;

    processor_config.required_start_frames =
      static_cast<std::size_t>(
      config_.vad_required_start_frames);

    processor_config.required_end_frames =
      static_cast<std::size_t>(
      config_.vad_required_end_frames);

    processor_config.event_queue_capacity =
      static_cast<std::size_t>(
      config_.vad_event_queue_capacity);

    if (!processor_config.is_valid()) {
      throw std::invalid_argument{
              "invalid vad.processor parameter combination"};
    }
  }

  if (config_.utterance_session_enabled) {
    if (!config_.audio_required) {
      throw std::invalid_argument{
              "utterance_session.enabled requires "
              "audio.required=true"};
    }

    if (!config_.wake_word_enabled) {
      throw std::invalid_argument{
              "utterance_session.enabled requires "
              "wake_word.enabled=true"};
    }

    if (!config_.vad_enabled) {
      throw std::invalid_argument{
              "utterance_session.enabled requires "
              "vad.enabled=true"};
    }

    if (
      config_.utterance_session_required &&
      !config_.wake_word_required)
    {
      throw std::invalid_argument{
              "required utterance session requires "
              "wake_word.required=true"};
    }

    if (
      config_.utterance_session_required &&
      !config_.vad_required)
    {
      throw std::invalid_argument{
              "required utterance session requires "
              "vad.required=true"};
    }

    validate_integer_range(
      config_.utterance_session_pre_roll_ms,
      1,
      10000,
      "utterance_session.pre_roll_ms");

    validate_integer_range(
      config_.utterance_session_speech_start_timeout_ms,
      1,
      60000,
      "utterance_session.speech_start_timeout_ms");

    validate_integer_range(
      config_.utterance_session_maximum_duration_ms,
      1,
      300000,
      "utterance_session.maximum_duration_ms");

    validate_integer_range(
      config_.utterance_session_completed_queue_capacity,
      1,
      1024,
      "utterance_session.completed_queue_capacity");

    session::UtteranceSessionConfig session_config;

    session_config.pre_roll_duration =
      std::chrono::milliseconds{
      config_.utterance_session_pre_roll_ms};

    session_config.speech_start_timeout =
      std::chrono::milliseconds{
      config_.utterance_session_speech_start_timeout_ms};

    session_config.maximum_utterance_duration =
      std::chrono::milliseconds{
      config_.utterance_session_maximum_duration_ms};

    session_config.completed_queue_capacity =
      static_cast<std::size_t>(
      config_.utterance_session_completed_queue_capacity);

    if (!session_config.is_valid()) {
      throw std::invalid_argument{
              "invalid utterance_session parameter "
              "combination"};
    }
  }

  if (config_.utterance_serialization_enabled) {
    if (!config_.utterance_session_enabled) {
      throw std::invalid_argument{
              "utterance_serialization.enabled requires "
              "utterance_session.enabled=true"};
    }

    if (
      config_.utterance_serialization_required &&
      !config_.utterance_session_required)
    {
      throw std::invalid_argument{
              "required utterance serialization requires "
              "utterance_session.required=true"};
    }

    validate_integer_range(
      config_.
      utterance_serialization_output_queue_capacity,
      1,
      1024,
      "utterance_serialization.output_queue_capacity");

    validate_integer_range(
      config_.
      utterance_serialization_source_wait_timeout_ms,
      1,
      5000,
      "utterance_serialization.source_wait_timeout_ms");

    validate_integer_range(
      config_.utterance_serialization_maximum_wav_bytes,
      44,
      64 * 1024 * 1024,
      "utterance_serialization.maximum_wav_bytes");

    session::CompletedUtteranceWorkerConfig worker_config;

    worker_config.output_queue_capacity =
      static_cast<std::size_t>(
      config_.
      utterance_serialization_output_queue_capacity);

    worker_config.source_wait_timeout =
      std::chrono::milliseconds{
      config_.
      utterance_serialization_source_wait_timeout_ms};

    worker_config.maximum_wav_bytes =
      static_cast<std::size_t>(
      config_.utterance_serialization_maximum_wav_bytes);

    if (!worker_config.is_valid()) {
      throw std::invalid_argument{
              "invalid utterance_serialization parameter "
              "combination"};
    }
  }

  static_cast<void>(
    pre_roll_samples(
      config_.sample_rate_hz,
      config_.pre_roll_ms));

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

  if (!config_.audio_required) {
    phase_ = session::SpeechPhase::Idle;
    error_ = session::SpeechError::None;
    ready_ = true;
    reason_ = "audio_not_required";
    return;
  }

  phase_ = session::SpeechPhase::WaitingForAudio;
  error_ = session::SpeechError::AudioNotInitialized;
  reason_ = "audio_runtime_starting";

  initialize_audio_runtime();
}

void SpeechNode::initialize_audio_runtime()
{
  try {
    drivers::AlsaCaptureConfig capture_config;

    capture_config.device_name =
      config_.capture_device;

    capture_config.requested_format = {
      static_cast<std::uint32_t>(
        config_.sample_rate_hz),
      static_cast<std::uint16_t>(
        config_.capture_channels),
      audio::PcmSampleFormat::
      Signed16LittleEndian};

    capture_config.period_frames =
      static_cast<std::size_t>(
      config_.period_frames);

    capture_config.periods =
      static_cast<std::size_t>(
      config_.periods);

    capture_config.require_exact_sample_rate =
      config_.require_exact_sample_rate;

    drivers::AlsaPlaybackConfig playback_config;

    playback_config.device_name =
      config_.playback_device;

    playback_config.requested_format = {
      static_cast<std::uint32_t>(
        config_.sample_rate_hz),
      static_cast<std::uint16_t>(
        config_.playback_channels),
      audio::PcmSampleFormat::
      Signed16LittleEndian};

    playback_config.period_frames =
      static_cast<std::size_t>(
      config_.period_frames);

    playback_config.periods =
      static_cast<std::size_t>(
      config_.periods);

    playback_config.require_exact_sample_rate =
      config_.require_exact_sample_rate;

    capture_stream_ =
      std::make_unique<drivers::AlsaCaptureStream>(
      capture_config);

    playback_stream_ =
      std::make_unique<drivers::AlsaPlaybackStream>(
      playback_config);

    audio::AudioRuntimeConfig runtime_config;

    runtime_config.microphone_gate.
    post_playback_hold =
      std::chrono::milliseconds{
      config_.post_playback_hold_ms};

    runtime_config.capture_pipeline.selected_channel =
      static_cast<std::uint16_t>(
      config_.selected_channel);

    runtime_config.capture_pipeline.pre_roll_samples =
      pre_roll_samples(
      config_.sample_rate_hz,
      config_.pre_roll_ms);

    runtime_config.capture_pipeline.
    queue_capacity_frames =
      static_cast<std::size_t>(
      config_.capture_queue_capacity_frames);

    runtime_config.capture_pipeline.
    queue_overflow_policy =
      audio::QueueOverflowPolicy::DropOldest;

    runtime_config.playback_worker.queue_capacity =
      static_cast<std::size_t>(
      config_.playback_queue_capacity);

    runtime_config.playback_worker.chunk_frames =
      static_cast<std::size_t>(
      config_.chunk_frames);

    audio_runtime_ =
      std::make_unique<audio::AudioRuntime>(
      *capture_stream_,
      *playback_stream_,
      runtime_config);

    const bool started = audio_runtime_->start();

    if (!started || !audio_runtime_->ready()) {
      throw std::runtime_error{
              "audio runtime did not enter the running state"};
    }

    audio_activity_monitor_ =
      std::make_unique<audio::AudioActivityMonitor>();

    captured_audio_processor_chain_ =
      std::make_unique<
      audio::CapturedAudioProcessorChain>();

    captured_audio_processor_chain_->add_processor(
      "audio_activity",
      *audio_activity_monitor_,
      true);

    wake_word_initialization_error_.clear();

    if (config_.wake_word_enabled) {
      try {
        wake_word::WakeWordAssetSelectionConfig
          asset_selection;

        asset_selection.package_share_directory =
          ament_index_cpp::get_package_share_directory(
          "savo_speech");

        asset_selection.profile =
          config_.wake_word_profile;

        asset_selection.dictionary_override =
          std::filesystem::path{
          config_.wake_word_dictionary_path};

        asset_selection.keyword_file_override =
          std::filesystem::path{
          config_.wake_word_keyword_file_path};

        const auto assets =
          wake_word::resolve_wake_word_assets(
          asset_selection);

        wake_word::PocketSphinxWakeWordBackendConfig
          backend_config;

        backend_config.acoustic_model_path =
          config_.wake_word_acoustic_model_path;

        backend_config.dictionary_path =
          assets.dictionary_path;

        backend_config.keyword_file_path =
          assets.keyword_file_path;

        backend_config.search_name =
          config_.wake_word_search_name;

        backend_config.sample_rate_hz =
          static_cast<std::uint32_t>(
          config_.sample_rate_hz);

        backend_config.suppress_decoder_log =
          config_.wake_word_suppress_decoder_log;

        wake_word_backend_ =
          std::make_unique<
          wake_word::PocketSphinxWakeWordBackend>(
          std::move(backend_config));

        wake_word::WakeWordProcessorConfig
          processor_config;

        processor_config.confidence_threshold =
          config_.wake_word_confidence_threshold;

        processor_config.required_consecutive_detections =
          static_cast<std::size_t>(
          config_.wake_word_consecutive_detections);

        processor_config.cooldown =
          std::chrono::milliseconds{
          config_.wake_word_cooldown_ms};

        processor_config.event_queue_capacity =
          static_cast<std::size_t>(
          config_.wake_word_event_queue_capacity);

        wake_word_processor_ =
          std::make_unique<
          wake_word::WakeWordProcessor>(
          *wake_word_backend_,
          processor_config);

        captured_audio_processor_chain_->add_processor(
          "wake_word",
          *wake_word_processor_,
          config_.wake_word_required);

        RCLCPP_INFO(
          get_logger(),
          "Wake-word backend ready: profile=%s "
          "required=%s dictionary=%s keywords=%s",
          config_.wake_word_profile.c_str(),
          bool_text(config_.wake_word_required).c_str(),
          assets.dictionary_path.string().c_str(),
          assets.keyword_file_path.string().c_str());
      } catch (const std::exception & exception) {
        wake_word_initialization_error_ =
          exception.what();

        wake_word_processor_.reset();
        wake_word_backend_.reset();

        if (config_.wake_word_required) {
          throw std::runtime_error{
                  "required wake-word initialization "
                  "failed: " +
                  wake_word_initialization_error_};
        }

        RCLCPP_WARN(
          get_logger(),
          "Optional wake-word backend unavailable: %s",
          wake_word_initialization_error_.c_str());
      }
    }

    vad_initialization_error_.clear();

    if (config_.vad_enabled) {
      try {
        vad::AdaptiveEnergyVadConfig backend_config;

        backend_config.expected_sample_rate_hz =
          static_cast<std::uint32_t>(
          config_.sample_rate_hz);

        backend_config.startup_calibration_frames =
          static_cast<std::size_t>(
          config_.vad_startup_calibration_frames);

        backend_config.initial_noise_floor_rms =
          config_.vad_initial_noise_floor_rms;

        backend_config.minimum_noise_floor_rms =
          config_.vad_minimum_noise_floor_rms;

        backend_config.maximum_noise_floor_rms =
          config_.vad_maximum_noise_floor_rms;

        backend_config.noise_floor_update_alpha =
          config_.vad_noise_floor_update_alpha;

        backend_config.minimum_speech_rms =
          config_.vad_minimum_speech_rms;

        backend_config.speech_onset_snr_db =
          config_.vad_speech_onset_snr_db;

        backend_config.speech_saturation_snr_db =
          config_.vad_speech_saturation_snr_db;

        backend_config.clipping_threshold =
          static_cast<std::uint32_t>(
          config_.vad_clipping_threshold);

        vad_backend_ =
          std::make_unique<
          vad::AdaptiveEnergyVadBackend>(
          backend_config);

        vad::VadProcessorConfig processor_config;

        processor_config.speech_start_threshold =
          config_.vad_speech_start_threshold;

        processor_config.speech_end_threshold =
          config_.vad_speech_end_threshold;

        processor_config.required_start_frames =
          static_cast<std::size_t>(
          config_.vad_required_start_frames);

        processor_config.required_end_frames =
          static_cast<std::size_t>(
          config_.vad_required_end_frames);

        processor_config.event_queue_capacity =
          static_cast<std::size_t>(
          config_.vad_event_queue_capacity);

        vad_processor_ =
          std::make_unique<vad::VadProcessor>(
          *vad_backend_,
          processor_config);

        captured_audio_processor_chain_->add_processor(
          "vad",
          *vad_processor_,
          config_.vad_required);

        RCLCPP_INFO(
          get_logger(),
          "Adaptive-energy VAD ready: required=%s",
          bool_text(config_.vad_required).c_str());
      } catch (const std::exception & exception) {
        vad_initialization_error_ =
          exception.what();

        vad_processor_.reset();
        vad_backend_.reset();

        if (config_.vad_required) {
          throw std::runtime_error{
                  "required VAD initialization failed: " +
                  vad_initialization_error_};
        }

        RCLCPP_WARN(
          get_logger(),
          "Optional VAD backend unavailable: %s",
          vad_initialization_error_.c_str());
      }
    }

    utterance_session_initialization_error_.clear();

    if (config_.utterance_session_enabled) {
      try {
        if (
          !wake_word_processor_ ||
          !vad_processor_)
        {
          throw std::runtime_error{
                  "utterance session requires initialized "
                  "wake-word and VAD processors"};
        }

        session::UtteranceSessionConfig session_config;

        session_config.pre_roll_duration =
          std::chrono::milliseconds{
          config_.utterance_session_pre_roll_ms};

        session_config.speech_start_timeout =
          std::chrono::milliseconds{
          config_.
          utterance_session_speech_start_timeout_ms};

        session_config.maximum_utterance_duration =
          std::chrono::milliseconds{
          config_.
          utterance_session_maximum_duration_ms};

        session_config.completed_queue_capacity =
          static_cast<std::size_t>(
          config_.
          utterance_session_completed_queue_capacity);

        utterance_session_processor_ =
          std::make_unique<
          session::UtteranceSessionProcessor>(
          *wake_word_processor_,
          *vad_processor_,
          session_config);

        captured_audio_processor_chain_->add_processor(
          "utterance_session",
          *utterance_session_processor_,
          config_.utterance_session_required);

        RCLCPP_INFO(
          get_logger(),
          "Utterance-session processor ready: "
          "required=%s pre_roll_ms=%ld "
          "speech_start_timeout_ms=%ld "
          "maximum_duration_ms=%ld",
          bool_text(
            config_.
            utterance_session_required).c_str(),
          static_cast<long>(
            config_.utterance_session_pre_roll_ms),
          static_cast<long>(
            config_.
            utterance_session_speech_start_timeout_ms),
          static_cast<long>(
            config_.
            utterance_session_maximum_duration_ms));
      } catch (const std::exception & exception) {
        utterance_session_initialization_error_ =
          exception.what();

        utterance_session_processor_.reset();

        if (config_.utterance_session_required) {
          throw std::runtime_error{
                  "required utterance-session "
                  "initialization failed: " +
                  utterance_session_initialization_error_};
        }

        RCLCPP_WARN(
          get_logger(),
          "Optional utterance-session processor "
          "unavailable: %s",
          utterance_session_initialization_error_.c_str());
      }
    }

    utterance_serialization_initialization_error_.clear();

    if (config_.utterance_serialization_enabled) {
      try {
        if (!utterance_session_processor_) {
          throw std::runtime_error{
                  "utterance serialization requires an "
                  "initialized utterance-session processor"};
        }

        session::CompletedUtteranceWorkerConfig worker_config;

        worker_config.output_queue_capacity =
          static_cast<std::size_t>(
          config_.
          utterance_serialization_output_queue_capacity);

        worker_config.source_wait_timeout =
          std::chrono::milliseconds{
          config_.
          utterance_serialization_source_wait_timeout_ms};

        worker_config.maximum_wav_bytes =
          static_cast<std::size_t>(
          config_.
          utterance_serialization_maximum_wav_bytes);

        completed_utterance_worker_ =
          std::make_unique<
          session::CompletedUtteranceWorker>(
          *utterance_session_processor_,
          worker_config);

        if (!completed_utterance_worker_->start()) {
          throw std::runtime_error{
                  "completed-utterance worker refused "
                  "to start"};
        }

        RCLCPP_INFO(
          get_logger(),
          "Completed-utterance worker ready: "
          "required=%s output_queue_capacity=%ld "
          "source_wait_timeout_ms=%ld "
          "maximum_wav_bytes=%ld",
          bool_text(
            config_.
            utterance_serialization_required).c_str(),
          static_cast<long>(
            config_.
            utterance_serialization_output_queue_capacity),
          static_cast<long>(
            config_.
            utterance_serialization_source_wait_timeout_ms),
          static_cast<long>(
            config_.
            utterance_serialization_maximum_wav_bytes));
      } catch (const std::exception & exception) {
        utterance_serialization_initialization_error_ =
          exception.what();

        if (completed_utterance_worker_) {
          completed_utterance_worker_->stop();
        }

        completed_utterance_worker_.reset();

        if (config_.utterance_serialization_required) {
          throw std::runtime_error{
                  "required utterance serialization "
                  "initialization failed: " +
                  utterance_serialization_initialization_error_};
        }

        RCLCPP_WARN(
          get_logger(),
          "Optional completed-utterance worker "
          "unavailable: %s",
          utterance_serialization_initialization_error_.
          c_str());
      }
    }

    const bool processor_chain_sealed =
      captured_audio_processor_chain_->seal();

    if (!processor_chain_sealed) {
      throw std::runtime_error{
              "captured-audio processor chain was "
              "already sealed"};
    }

    audio::CaptureProcessingConfig processing_config;

    processing_config.wait_timeout =
      std::chrono::milliseconds{
      config_.processing_wait_timeout_ms};

    processing_config.freshness_timeout =
      std::chrono::milliseconds{
      config_.processing_freshness_timeout_ms};

    processing_config.fault_on_processor_error =
      config_.processing_fault_on_processor_error;

    capture_processing_dispatcher_ =
      std::make_unique<
      audio::CaptureProcessingDispatcher>(
      *audio_runtime_,
      *captured_audio_processor_chain_,
      processing_config);

    const bool processing_started =
      capture_processing_dispatcher_->start();

    if (!processing_started) {
      throw std::runtime_error{
              "capture processing dispatcher was already active"};
    }

    refresh_runtime_state();

    RCLCPP_INFO(
      get_logger(),
      "Audio runtime ready: capture=%s playback=%s "
      "rate=%ld capture_channels=%ld playback_channels=%ld "
      "selected_channel=%ld period_frames=%ld",
      config_.capture_device.c_str(),
      config_.playback_device.c_str(),
      static_cast<long>(config_.sample_rate_hz),
      static_cast<long>(config_.capture_channels),
      static_cast<long>(config_.playback_channels),
      static_cast<long>(config_.selected_channel),
      static_cast<long>(config_.period_frames));
  } catch (const std::exception & exception) {
    shutdown_audio_runtime();

    ready_ = false;
    audio_initialized_ = false;

    phase_ = session::SpeechPhase::Error;
    error_ = classify_audio_error(exception.what());

    reason_ =
      "audio_runtime_start_failed: " +
      std::string{exception.what()};

    RCLCPP_ERROR(
      get_logger(),
      "Failed to initialize required audio runtime: %s",
      exception.what());
  }
}

void SpeechNode::shutdown_audio_runtime() noexcept
{
  if (capture_processing_dispatcher_) {
    capture_processing_dispatcher_->stop();
  }

  if (completed_utterance_worker_) {
    completed_utterance_worker_->stop();
  }

  capture_processing_dispatcher_.reset();
  completed_utterance_worker_.reset();

  captured_audio_processor_chain_.reset();

  utterance_session_processor_.reset();

  vad_processor_.reset();
  vad_backend_.reset();

  wake_word_processor_.reset();
  wake_word_backend_.reset();

  audio_activity_monitor_.reset();

  if (audio_runtime_) {
    audio_runtime_->stop();
  }

  audio_runtime_.reset();
  playback_stream_.reset();
  capture_stream_.reset();

  audio_initialized_ = false;
}

void SpeechNode::refresh_runtime_state()
{
  if (!config_.enabled || !config_.audio_required) {
    return;
  }

  if (!audio_runtime_) {
    ready_ = false;
    audio_initialized_ = false;

    if (phase_ != session::SpeechPhase::Error) {
      phase_ = session::SpeechPhase::WaitingForAudio;
      error_ = session::SpeechError::AudioNotInitialized;
      reason_ = "audio_runtime_not_constructed";
    }

    return;
  }

  const auto snapshot =
    audio_runtime_->health_snapshot();

  switch (snapshot.state) {
    case audio::AudioRuntimeState::Running:
      {
        if (
          !capture_processing_dispatcher_ ||
          !audio_activity_monitor_)
        {
          ready_ = false;
          audio_initialized_ = snapshot.ready;

          phase_ = session::SpeechPhase::Error;
          error_ =
            session::SpeechError::AudioProcessingFailed;

          reason_ =
            "capture_processing_not_constructed";

          return;
        }

        const auto processing =
          capture_processing_dispatcher_->
          health_snapshot();

        if (
          processing.state ==
          audio::CaptureProcessingState::Faulted)
        {
          ready_ = false;
          audio_initialized_ = snapshot.ready;

          phase_ = session::SpeechPhase::Error;
          error_ =
            session::SpeechError::AudioProcessingFailed;

          reason_ =
            processing.last_error.empty() ?
            "capture_processing_faulted" :
            processing.last_error;

          return;
        }

        if (
          processing.state !=
          audio::CaptureProcessingState::Running)
        {
          ready_ = false;
          audio_initialized_ = snapshot.ready;

          phase_ =
            session::SpeechPhase::WaitingForAudio;

          error_ =
            session::SpeechError::AudioNotInitialized;

          reason_ =
            "capture_processing_not_running";

          return;
        }

        if (
          processing.statistics.frames_processed == 0U)
        {
          ready_ = false;
          audio_initialized_ = snapshot.ready;

          phase_ =
            session::SpeechPhase::WaitingForAudio;

          error_ =
            session::SpeechError::AudioNotInitialized;

          reason_ =
            "waiting_for_first_processed_audio_frame";

          return;
        }

        const bool processing_freshness_suspended =
          snapshot.microphone_gate.gated;

        if (
          !processing.fresh &&
          !processing_freshness_suspended)
        {
          ready_ = false;
          audio_initialized_ = snapshot.ready;

          phase_ = session::SpeechPhase::Error;
          error_ =
            session::SpeechError::AudioProcessingFailed;

          reason_ = "capture_processing_stale";

          return;
        }

        ready_ = snapshot.ready;
        audio_initialized_ = snapshot.ready;

        error_ = session::SpeechError::None;
        reason_ = "audio_runtime_ready";

        switch (snapshot.microphone_gate.reason) {
          case audio::MicrophoneGateReason::Playback:
            phase_ = session::SpeechPhase::Speaking;
            break;

          case audio::MicrophoneGateReason::Manual:
            phase_ = session::SpeechPhase::Muted;
            break;

          case audio::MicrophoneGateReason::Open:
          case audio::MicrophoneGateReason::
            PostPlaybackHold:
          case audio::MicrophoneGateReason::Shutdown:
            if (!utterance_session_processor_) {
              phase_ = session::SpeechPhase::Idle;
              break;
            }

            switch (
              utterance_session_processor_->
              snapshot().session.state)
            {
              case session::UtteranceSessionState::Idle:
                phase_ = session::SpeechPhase::Idle;
                break;

              case session::UtteranceSessionState::Armed:
                phase_ = session::SpeechPhase::Listening;
                break;

              case session::UtteranceSessionState::Recording:
                phase_ = session::SpeechPhase::Recording;
                break;
            }

            break;
        }

        return;
      }

    case audio::AudioRuntimeState::Starting:
      ready_ = false;
      audio_initialized_ = false;
      phase_ = session::SpeechPhase::WaitingForAudio;
      error_ = session::SpeechError::AudioNotInitialized;
      reason_ = "audio_runtime_starting";
      return;

    case audio::AudioRuntimeState::Stopping:
    case audio::AudioRuntimeState::Stopped:
      ready_ = false;
      audio_initialized_ = false;
      phase_ = session::SpeechPhase::WaitingForAudio;
      error_ = session::SpeechError::AudioNotInitialized;
      reason_ = "audio_runtime_stopped";
      return;

    case audio::AudioRuntimeState::Faulted:
      ready_ = false;
      audio_initialized_ = false;
      phase_ = session::SpeechPhase::Error;

      reason_ =
        snapshot.last_error.empty() ?
        "audio_runtime_faulted" :
        snapshot.last_error;

      error_ = classify_audio_error(reason_);
      return;
  }

  ready_ = false;
  audio_initialized_ = false;
  phase_ = session::SpeechPhase::Error;
  error_ = session::SpeechError::InternalError;
  reason_ = "unknown_audio_runtime_state";
}

void SpeechNode::publish_runtime_status()
{
  refresh_runtime_state();

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
    << "profile=" << config_.profile
    << " enabled=" << bool_text(config_.enabled)
    << " phase=" << session::to_string(phase_)
    << " ready=" << bool_text(ready_)
    << " audio=" << bool_text(audio_initialized_)
    << " savomind=" << bool_text(savomind_initialized_)
    << " error=" << session::to_string(error_)
    << " reason=" << reason_
    << " capture_device=" << config_.capture_device
    << " playback_device=" << config_.playback_device;

  if (audio_runtime_) {
    const auto snapshot =
      audio_runtime_->health_snapshot();

    stream
      << " runtime="
      << audio::to_string(snapshot.state)
      << " capture_worker="
      << audio::to_string(
        snapshot.capture_worker_state)
      << " playback_worker="
      << audio::to_string(
        snapshot.playback_worker_state)
      << " gate="
      << audio::to_string(
        snapshot.microphone_gate.reason)
      << " capture_frames="
      << snapshot.capture_worker_statistics.frames_read
      << " capture_accepted="
      << snapshot.capture_worker_statistics.accepted_frames
      << " capture_gated="
      << snapshot.capture_worker_statistics.gated_frames
      << " capture_queue_drops="
      << snapshot.capture_worker_statistics.queue_drop_events
      << " playback_pending="
      << snapshot.pending_playback_requests
      << " playback_completed="
      << snapshot.playback_worker_statistics.completed_requests
      << " playback_cancelled="
      << snapshot.playback_worker_statistics.cancelled_requests
      << " playback_failed="
      << snapshot.playback_worker_statistics.failed_requests;
  }

  if (capture_processing_dispatcher_) {
    const auto processing =
      capture_processing_dispatcher_->
      health_snapshot();

    stream
      << " processing_state="
      << audio::to_string(processing.state)
      << " processing_ready="
      << bool_text(processing.ready)
      << " processing_fresh="
      << bool_text(processing.fresh)
      << " processing_age_ms="
      << processing.last_frame_age_ms
      << " processing_received="
      << processing.statistics.frames_received
      << " processing_frames="
      << processing.statistics.frames_processed
      << " processing_timeouts="
      << processing.statistics.wait_timeouts
      << " processing_failures="
      << processing.statistics.processor_failures
      << " sequence_gaps="
      << processing.statistics.missing_sequence_frames
      << " out_of_order="
      << processing.statistics.out_of_order_frames;
  }

  if (captured_audio_processor_chain_) {
    const auto chain =
      captured_audio_processor_chain_->snapshot();

    stream
      << " chain_sealed="
      << bool_text(chain.sealed)
      << " chain_processors="
      << chain.processor_count
      << " chain_frames_received="
      << chain.statistics.frames_received
      << " chain_frames_completed="
      << chain.statistics.frames_completed
      << " chain_frames_failed="
      << chain.statistics.frames_failed
      << " chain_optional_failure_frames="
      << chain.statistics.
      frames_with_optional_failures
      << " chain_required_failures="
      << chain.statistics.
      required_processor_failures
      << " chain_optional_failures="
      << chain.statistics.
      optional_processor_failures;
  }

  if (audio_activity_monitor_) {
    const auto activity =
      audio_activity_monitor_->snapshot();

    stream
      << " audio_rms="
      << activity.last_rms
      << " audio_peak="
      << activity.last_peak
      << " audio_max_peak="
      << activity.maximum_peak
      << " clipping_frames="
      << activity.clipping_frames;
  }

  stream
    << " wake_word_enabled="
    << bool_text(config_.wake_word_enabled)
    << " wake_word_required="
    << bool_text(config_.wake_word_required)
    << " wake_word_profile="
    << config_.wake_word_profile;

  if (wake_word_backend_) {
    const auto backend =
      wake_word_backend_->snapshot();

    stream
      << " wake_backend_ready="
      << bool_text(
        backend.initialized &&
        backend.stream_started &&
        backend.utterance_active)
      << " wake_backend_frames="
      << backend.statistics.frames_analyzed
      << " wake_backend_detections="
      << backend.statistics.detections
      << " wake_backend_restarts="
      << backend.statistics.decoder_restarts
      << " wake_backend_failures="
      << backend.statistics.process_failures;
  }

  if (wake_word_processor_) {
    const auto wake =
      wake_word_processor_->snapshot();

    stream
      << " wake_frames="
      << wake.statistics.frames_processed
      << " wake_accepted="
      << wake.statistics.accepted_detections
      << " wake_suppressed="
      << wake.statistics.cooldown_suppressed
      << " wake_queued_events="
      << wake.queued_events
      << " wake_events_dropped="
      << wake.statistics.events_dropped
      << " wake_last_phrase="
      << wake.last_detected_phrase;
  }

  if (!wake_word_initialization_error_.empty()) {
    stream
      << " wake_initialization_error="
      << wake_word_initialization_error_;
  }

  stream
    << " vad_enabled="
    << bool_text(config_.vad_enabled)
    << " vad_required="
    << bool_text(config_.vad_required);

  if (vad_backend_) {
    const auto backend =
      vad_backend_->snapshot();

    stream
      << " vad_calibrated="
      << bool_text(backend.calibrated)
      << " vad_calibration_frames="
      << backend.calibration_frames_completed
      << " vad_noise_floor_rms="
      << backend.noise_floor_rms
      << " vad_rms="
      << backend.last_rms
      << " vad_peak="
      << backend.last_peak
      << " vad_snr_db="
      << backend.last_snr_db
      << " vad_speech_score="
      << backend.last_speech_score
      << " vad_clipping="
      << bool_text(backend.last_frame_clipping)
      << " vad_backend_frames="
      << backend.statistics.frames_analyzed
      << " vad_noise_updates="
      << backend.statistics.noise_floor_updates
      << " vad_noise_freezes="
      << backend.statistics.noise_floor_freezes;
  }

  if (vad_processor_) {
    const auto processor =
      vad_processor_->snapshot();

    stream
      << " vad_state="
      << vad::to_string(processor.state)
      << " vad_processor_frames="
      << processor.statistics.frames_processed
      << " vad_speech_started="
      << processor.statistics.speech_started_events
      << " vad_speech_ended="
      << processor.statistics.speech_ended_events
      << " vad_active_segment="
      << processor.active_segment_id
      << " vad_queued_events="
      << processor.queued_events
      << " vad_events_dropped="
      << processor.statistics.events_dropped;
  }

  if (!vad_initialization_error_.empty()) {
    stream
      << " vad_initialization_error="
      << vad_initialization_error_;
  }

  stream
    << " utterance_session_enabled="
    << bool_text(config_.utterance_session_enabled)
    << " utterance_session_required="
    << bool_text(config_.utterance_session_required);

  if (utterance_session_processor_) {
    const auto utterance =
      utterance_session_processor_->snapshot();

    const std::string last_completion =
      utterance.session.last_completion_reason.has_value() ?
      std::string{
      session::to_string(
          *utterance.session.last_completion_reason)} :
    std::string{"none"};

    stream
      << " utterance_state="
      << session::to_string(
        utterance.session.state)
      << " utterance_active_id="
      << utterance.session.active_utterance_id
      << " utterance_active_segment="
      << utterance.session.active_vad_segment_id
      << " utterance_wake_phrase="
      << utterance.session.active_wake_phrase
      << " utterance_active_samples="
      << utterance.session.active_audio_samples
      << " utterance_queued_completed="
      << utterance.session.queued_completed_utterances
      << " utterance_processor_frames="
      << utterance.statistics.frames_received
      << " utterance_processor_failures="
      << utterance.statistics.frames_failed
      << " utterance_wake_accepted="
      << utterance.statistics.wake_events_accepted
      << " utterance_wake_rejected="
      << utterance.statistics.wake_events_rejected
      << " utterance_vad_accepted="
      << utterance.statistics.vad_events_accepted
      << " utterance_vad_rejected="
      << utterance.statistics.vad_events_rejected
      << " utterances_completed="
      << utterance.session.statistics.utterances_completed
      << " utterance_sessions_canceled="
      << utterance.session.statistics.sessions_canceled
      << " utterance_sequence_gaps="
      << utterance.session.statistics.sequence_gaps
      << " utterance_missing_frames="
      << utterance.session.statistics.missing_audio_frames
      << " utterance_last_cancellation="
      << session::to_string(
        utterance.session.last_cancellation_reason)
      << " utterance_last_completion="
      << last_completion;

    if (!utterance.last_error.empty()) {
      stream
        << " utterance_processor_error="
        << utterance.last_error;
    }

    if (!utterance.session.last_error.empty()) {
      stream
        << " utterance_session_error="
        << utterance.session.last_error;
    }
  }

  if (
    !utterance_session_initialization_error_.empty())
  {
    stream
      << " utterance_initialization_error="
      << utterance_session_initialization_error_;
  }

  stream
    << " utterance_serialization_enabled="
    << bool_text(config_.utterance_serialization_enabled)
    << " utterance_serialization_required="
    << bool_text(config_.utterance_serialization_required);

  if (completed_utterance_worker_) {
    const auto serialization =
      completed_utterance_worker_->snapshot();

    const std::string current_id =
      serialization.current_utterance_id.has_value() ?
      std::to_string(
        *serialization.current_utterance_id) :
      std::string{"none"};

    const std::string last_id =
      serialization.last_seen_utterance_id.has_value() ?
      std::to_string(
        *serialization.last_seen_utterance_id) :
      std::string{"none"};

    stream
      << " utterance_serialization_state="
      << session::to_string(serialization.state)
      << " serialized_queue_size="
      << serialization.output_queue.size
      << " serialized_queue_capacity="
      << serialization.output_queue.capacity
      << " serialization_received="
      << serialization.statistics.utterances_received
      << " serialization_completed="
      << serialization.statistics.utterances_serialized
      << " serialization_invalid="
      << serialization.statistics.invalid_utterances
      << " serialization_id_rejections="
      << serialization.statistics.
      duplicate_or_out_of_order_ids
      << " serialization_encoding_failures="
      << serialization.statistics.encoding_failures
      << " serialization_size_rejections="
      << serialization.statistics.
      wav_size_limit_rejections
      << " serialization_queue_rejections="
      << serialization.statistics.
      output_queue_rejections
      << " serialization_faults="
      << serialization.statistics.faults
      << " serialization_current_id="
      << current_id
      << " serialization_last_id="
      << last_id;

    if (!serialization.last_error.empty()) {
      stream
        << " serialization_error="
        << serialization.last_error;
    }
  }

  if (
    !utterance_serialization_initialization_error_.empty())
  {
    stream
      << " serialization_initialization_error="
      << utterance_serialization_initialization_error_;
  }

  return stream.str();
}

diagnostic_msgs::msg::DiagnosticArray
SpeechNode::create_diagnostics() const
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus runtime_status;

  runtime_status.name = "savo_speech/runtime";
  runtime_status.hardware_id = config_.device_id;

  if (!config_.enabled) {
    runtime_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    runtime_status.message = "package_disabled";
  } else if (ready_) {
    runtime_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::OK;

    runtime_status.message = "ready";
  } else if (phase_ == session::SpeechPhase::Error) {
    runtime_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    runtime_status.message = reason_;
  } else {
    runtime_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    runtime_status.message = reason_;
  }

  runtime_status.values.push_back(
    make_key_value(
      "version",
      owned_string(version::kVersion)));

  runtime_status.values.push_back(
    make_key_value(
      "profile",
      config_.profile));

  runtime_status.values.push_back(
    make_key_value(
      "robot_id",
      config_.robot_id));

  runtime_status.values.push_back(
    make_key_value(
      "host_role",
      config_.host_role));

  runtime_status.values.push_back(
    make_key_value(
      "phase",
      owned_string(session::to_string(phase_))));

  runtime_status.values.push_back(
    make_key_value(
      "ready",
      bool_text(ready_)));

  runtime_status.values.push_back(
    make_key_value(
      "audio_initialized",
      bool_text(audio_initialized_)));

  runtime_status.values.push_back(
    make_key_value(
      "savomind_initialized",
      bool_text(savomind_initialized_)));

  runtime_status.values.push_back(
    make_key_value(
      "error",
      owned_string(session::to_string(error_))));

  runtime_status.values.push_back(
    make_key_value(
      "reason",
      reason_));

  array.status.push_back(
    std::move(runtime_status));

  diagnostic_msgs::msg::DiagnosticStatus audio_status;

  audio_status.name = "savo_speech/audio";
  audio_status.hardware_id = config_.device_id;

  audio_status.values.push_back(
    make_key_value(
      "required",
      bool_text(config_.audio_required)));

  audio_status.values.push_back(
    make_key_value(
      "capture_device",
      config_.capture_device));

  audio_status.values.push_back(
    make_key_value(
      "playback_device",
      config_.playback_device));

  audio_status.values.push_back(
    make_key_value(
      "sample_rate_hz",
      std::to_string(config_.sample_rate_hz)));

  audio_status.values.push_back(
    make_key_value(
      "capture_channels",
      std::to_string(config_.capture_channels)));

  audio_status.values.push_back(
    make_key_value(
      "playback_channels",
      std::to_string(config_.playback_channels)));

  audio_status.values.push_back(
    make_key_value(
      "selected_channel",
      std::to_string(config_.selected_channel)));

  if (!config_.audio_required) {
    audio_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    audio_status.message = "audio_not_required";
  } else if (!audio_runtime_) {
    audio_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    audio_status.message = reason_;
  } else {
    const auto snapshot =
      audio_runtime_->health_snapshot();

    audio_status.level =
      diagnostic_level(snapshot.state);

    audio_status.message =
      owned_string(audio::to_string(snapshot.state));

    audio_status.values.push_back(
      make_key_value(
        "runtime_ready",
        bool_text(snapshot.ready)));

    audio_status.values.push_back(
      make_key_value(
        "capture_worker",
        owned_string(
          audio::to_string(
            snapshot.capture_worker_state))));

    audio_status.values.push_back(
      make_key_value(
        "playback_worker",
        owned_string(
          audio::to_string(
            snapshot.playback_worker_state))));

    audio_status.values.push_back(
      make_key_value(
        "microphone_gate",
        owned_string(
          audio::to_string(
            snapshot.microphone_gate.reason))));

    audio_status.values.push_back(
      make_key_value(
        "capture_frames_read",
        std::to_string(
          snapshot.capture_worker_statistics.frames_read)));

    audio_status.values.push_back(
      make_key_value(
        "capture_frames_accepted",
        std::to_string(
          snapshot.capture_worker_statistics.
          accepted_frames)));

    audio_status.values.push_back(
      make_key_value(
        "capture_frames_gated",
        std::to_string(
          snapshot.capture_worker_statistics.
          gated_frames)));

    audio_status.values.push_back(
      make_key_value(
        "capture_queue_drop_events",
        std::to_string(
          snapshot.capture_worker_statistics.
          queue_drop_events)));

    audio_status.values.push_back(
      make_key_value(
        "capture_faults",
        std::to_string(
          snapshot.capture_worker_statistics.faults)));

    audio_status.values.push_back(
      make_key_value(
        "playback_pending",
        std::to_string(
          snapshot.pending_playback_requests)));

    audio_status.values.push_back(
      make_key_value(
        "playback_completed",
        std::to_string(
          snapshot.playback_worker_statistics.
          completed_requests)));

    audio_status.values.push_back(
      make_key_value(
        "playback_cancelled",
        std::to_string(
          snapshot.playback_worker_statistics.
          cancelled_requests)));

    audio_status.values.push_back(
      make_key_value(
        "playback_failed",
        std::to_string(
          snapshot.playback_worker_statistics.
          failed_requests)));

    audio_status.values.push_back(
      make_key_value(
        "last_error",
        snapshot.last_error));
  }

  array.status.push_back(
    std::move(audio_status));

  diagnostic_msgs::msg::DiagnosticStatus
    processing_status;

  processing_status.name =
    "savo_speech/processing";

  processing_status.hardware_id =
    config_.device_id;

  processing_status.values.push_back(
    make_key_value(
      "wait_timeout_ms",
      std::to_string(
        config_.processing_wait_timeout_ms)));

  processing_status.values.push_back(
    make_key_value(
      "freshness_timeout_ms",
      std::to_string(
        config_.processing_freshness_timeout_ms)));

  if (!config_.enabled) {
    processing_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    processing_status.message = "package_disabled";
  } else if (!config_.audio_required) {
    processing_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    processing_status.message = "audio_not_required";
  } else if (
    !capture_processing_dispatcher_ ||
    !audio_activity_monitor_)
  {
    processing_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    processing_status.message =
      "capture_processing_not_constructed";
  } else {
    const auto processing =
      capture_processing_dispatcher_->
      health_snapshot();

    const auto activity =
      audio_activity_monitor_->snapshot();

    bool microphone_gated{false};

    if (audio_runtime_) {
      microphone_gated =
        audio_runtime_->health_snapshot().
        microphone_gate.gated;
    }

    if (
      processing.state ==
      audio::CaptureProcessingState::Faulted)
    {
      processing_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;

      processing_status.message =
        processing.last_error.empty() ?
        "faulted" :
        processing.last_error;
    } else if (
      processing.state ==
      audio::CaptureProcessingState::Running &&
      processing.statistics.frames_processed == 0U)
    {
      processing_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::WARN;

      processing_status.message =
        "waiting_for_first_frame";
    } else if (
      processing.state ==
      audio::CaptureProcessingState::Running &&
      (processing.fresh || microphone_gated))
    {
      processing_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::OK;

      processing_status.message =
        microphone_gated ?
        "running_microphone_gated" :
        "running";
    } else if (
      processing.state ==
      audio::CaptureProcessingState::Running)
    {
      processing_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;

      processing_status.message = "stale";
    } else {
      processing_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::WARN;

      processing_status.message =
        owned_string(
          audio::to_string(processing.state));
    }

    processing_status.values.push_back(
      make_key_value(
        "state",
        owned_string(
          audio::to_string(processing.state))));

    processing_status.values.push_back(
      make_key_value(
        "ready",
        bool_text(processing.ready)));

    processing_status.values.push_back(
      make_key_value(
        "fresh",
        bool_text(processing.fresh)));

    processing_status.values.push_back(
      make_key_value(
        "last_frame_age_ms",
        std::to_string(
          processing.last_frame_age_ms)));

    processing_status.values.push_back(
      make_key_value(
        "frames_received",
        std::to_string(
          processing.statistics.frames_received)));

    processing_status.values.push_back(
      make_key_value(
        "frames_processed",
        std::to_string(
          processing.statistics.frames_processed)));

    processing_status.values.push_back(
      make_key_value(
        "wait_timeouts",
        std::to_string(
          processing.statistics.wait_timeouts)));

    processing_status.values.push_back(
      make_key_value(
        "processor_failures",
        std::to_string(
          processing.statistics.processor_failures)));

    processing_status.values.push_back(
      make_key_value(
        "source_failures",
        std::to_string(
          processing.statistics.source_failures)));

    processing_status.values.push_back(
      make_key_value(
        "sequence_gaps",
        std::to_string(
          processing.statistics.
          missing_sequence_frames)));

    processing_status.values.push_back(
      make_key_value(
        "out_of_order_frames",
        std::to_string(
          processing.statistics.
          out_of_order_frames)));

    processing_status.values.push_back(
      make_key_value(
        "last_error",
        processing.last_error));

    processing_status.values.push_back(
      make_key_value(
        "audio_rms",
        std::to_string(activity.last_rms)));

    processing_status.values.push_back(
      make_key_value(
        "audio_peak",
        std::to_string(activity.last_peak)));

    processing_status.values.push_back(
      make_key_value(
        "maximum_peak",
        std::to_string(activity.maximum_peak)));

    processing_status.values.push_back(
      make_key_value(
        "clipping_frames",
        std::to_string(
          activity.clipping_frames)));
  }

  if (captured_audio_processor_chain_) {
    const auto chain =
      captured_audio_processor_chain_->snapshot();

    processing_status.values.push_back(
      make_key_value(
        "chain_sealed",
        bool_text(chain.sealed)));

    processing_status.values.push_back(
      make_key_value(
        "chain_processor_count",
        std::to_string(chain.processor_count)));

    processing_status.values.push_back(
      make_key_value(
        "chain_frames_received",
        std::to_string(
          chain.statistics.frames_received)));

    processing_status.values.push_back(
      make_key_value(
        "chain_frames_completed",
        std::to_string(
          chain.statistics.frames_completed)));

    processing_status.values.push_back(
      make_key_value(
        "chain_frames_failed",
        std::to_string(
          chain.statistics.frames_failed)));

    processing_status.values.push_back(
      make_key_value(
        "chain_required_failures",
        std::to_string(
          chain.statistics.
          required_processor_failures)));

    processing_status.values.push_back(
      make_key_value(
        "chain_optional_failures",
        std::to_string(
          chain.statistics.
          optional_processor_failures)));

    processing_status.values.push_back(
      make_key_value(
        "chain_last_error",
        chain.last_error));

    for (const auto & processor : chain.processors) {
      const std::string prefix =
        "processor." + processor.name + ".";

      processing_status.values.push_back(
        make_key_value(
          prefix + "required",
          bool_text(processor.required)));

      processing_status.values.push_back(
        make_key_value(
          prefix + "invocations",
          std::to_string(processor.invocations)));

      processing_status.values.push_back(
        make_key_value(
          prefix + "successes",
          std::to_string(processor.successes)));

      processing_status.values.push_back(
        make_key_value(
          prefix + "failures",
          std::to_string(processor.failures)));

      processing_status.values.push_back(
        make_key_value(
          prefix + "last_sequence",
          std::to_string(processor.last_sequence)));

      processing_status.values.push_back(
        make_key_value(
          prefix + "last_error",
          processor.last_error));
    }
  }

  array.status.push_back(
    std::move(processing_status));

  diagnostic_msgs::msg::DiagnosticStatus
    wake_word_status;

  wake_word_status.name =
    "savo_speech/wake_word";

  wake_word_status.hardware_id =
    config_.device_id;

  wake_word_status.values.push_back(
    make_key_value(
      "enabled",
      bool_text(config_.wake_word_enabled)));

  wake_word_status.values.push_back(
    make_key_value(
      "required",
      bool_text(config_.wake_word_required)));

  wake_word_status.values.push_back(
    make_key_value(
      "profile",
      config_.wake_word_profile));

  wake_word_status.values.push_back(
    make_key_value(
      "initialization_error",
      wake_word_initialization_error_));

  if (!config_.wake_word_enabled) {
    wake_word_status.level =
      diagnostic_msgs::msg::
      DiagnosticStatus::OK;

    wake_word_status.message = "disabled";
  } else if (
    !wake_word_backend_ ||
    !wake_word_processor_)
  {
    wake_word_status.level =
      config_.wake_word_required ?
      diagnostic_msgs::msg::
      DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::
      DiagnosticStatus::WARN;

    wake_word_status.message = "not_initialized";
  } else {
    const auto backend =
      wake_word_backend_->snapshot();

    const auto processor =
      wake_word_processor_->snapshot();

    const bool backend_ready =
      backend.initialized &&
      backend.stream_started &&
      backend.utterance_active;

    const bool healthy =
      backend_ready &&
      backend.last_error.empty() &&
      processor.last_error.empty();

    wake_word_status.level =
      healthy ?
      diagnostic_msgs::msg::
      DiagnosticStatus::OK :
      (
      config_.wake_word_required ?
      diagnostic_msgs::msg::
      DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::
      DiagnosticStatus::WARN
      );

    wake_word_status.message =
      healthy ? "running" : "degraded";

    wake_word_status.values.push_back(
      make_key_value(
        "backend_initialized",
        bool_text(backend.initialized)));

    wake_word_status.values.push_back(
      make_key_value(
        "stream_started",
        bool_text(backend.stream_started)));

    wake_word_status.values.push_back(
      make_key_value(
        "utterance_active",
        bool_text(backend.utterance_active)));

    wake_word_status.values.push_back(
      make_key_value(
        "active_search",
        backend.active_search));

    wake_word_status.values.push_back(
      make_key_value(
        "frames_analyzed",
        std::to_string(
          backend.statistics.frames_analyzed)));

    wake_word_status.values.push_back(
      make_key_value(
        "samples_processed",
        std::to_string(
          backend.statistics.samples_processed)));

    wake_word_status.values.push_back(
      make_key_value(
        "backend_detections",
        std::to_string(
          backend.statistics.detections)));

    wake_word_status.values.push_back(
      make_key_value(
        "decoder_restarts",
        std::to_string(
          backend.statistics.decoder_restarts)));

    wake_word_status.values.push_back(
      make_key_value(
        "process_failures",
        std::to_string(
          backend.statistics.process_failures)));

    wake_word_status.values.push_back(
      make_key_value(
        "restart_failures",
        std::to_string(
          backend.statistics.restart_failures)));

    wake_word_status.values.push_back(
      make_key_value(
        "backend_last_error",
        backend.last_error));

    wake_word_status.values.push_back(
      make_key_value(
        "processor_frames",
        std::to_string(
          processor.statistics.frames_processed)));

    wake_word_status.values.push_back(
      make_key_value(
        "accepted_detections",
        std::to_string(
          processor.statistics.accepted_detections)));

    wake_word_status.values.push_back(
      make_key_value(
        "cooldown_suppressed",
        std::to_string(
          processor.statistics.cooldown_suppressed)));

    wake_word_status.values.push_back(
      make_key_value(
        "queued_events",
        std::to_string(
          processor.queued_events)));

    wake_word_status.values.push_back(
      make_key_value(
        "events_dropped",
        std::to_string(
          processor.statistics.events_dropped)));

    wake_word_status.values.push_back(
      make_key_value(
        "last_detected_phrase",
        processor.last_detected_phrase));

    wake_word_status.values.push_back(
      make_key_value(
        "last_detected_confidence",
        std::to_string(
          processor.last_detected_confidence)));

    wake_word_status.values.push_back(
      make_key_value(
        "processor_last_error",
        processor.last_error));
  }

  array.status.push_back(
    std::move(wake_word_status));

  diagnostic_msgs::msg::DiagnosticStatus vad_status;

  vad_status.name = "savo_speech/vad";
  vad_status.hardware_id = config_.device_id;

  vad_status.values.push_back(
    make_key_value(
      "enabled",
      bool_text(config_.vad_enabled)));

  vad_status.values.push_back(
    make_key_value(
      "required",
      bool_text(config_.vad_required)));

  vad_status.values.push_back(
    make_key_value(
      "initialization_error",
      vad_initialization_error_));

  if (!config_.vad_enabled) {
    vad_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::OK;

    vad_status.message = "disabled";
  } else if (!vad_backend_ || !vad_processor_) {
    vad_status.level =
      config_.vad_required ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    vad_status.message = "not_initialized";
  } else {
    const auto backend =
      vad_backend_->snapshot();

    const auto processor =
      vad_processor_->snapshot();

    const bool healthy =
      backend.last_error.empty() &&
      processor.last_error.empty();

    if (!healthy) {
      vad_status.level =
        config_.vad_required ?
        diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        diagnostic_msgs::msg::DiagnosticStatus::WARN;

      vad_status.message = "degraded";
    } else if (!backend.calibrated) {
      vad_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::WARN;

      vad_status.message = "calibrating";
    } else {
      vad_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::OK;

      vad_status.message = "running";
    }

    vad_status.values.push_back(
      make_key_value(
        "calibrated",
        bool_text(backend.calibrated)));

    vad_status.values.push_back(
      make_key_value(
        "calibration_frames_completed",
        std::to_string(
          backend.calibration_frames_completed)));

    vad_status.values.push_back(
      make_key_value(
        "noise_floor_rms",
        std::to_string(
          backend.noise_floor_rms)));

    vad_status.values.push_back(
      make_key_value(
        "last_rms",
        std::to_string(
          backend.last_rms)));

    vad_status.values.push_back(
      make_key_value(
        "last_peak",
        std::to_string(
          backend.last_peak)));

    vad_status.values.push_back(
      make_key_value(
        "last_snr_db",
        std::to_string(
          backend.last_snr_db)));

    vad_status.values.push_back(
      make_key_value(
        "last_speech_score",
        std::to_string(
          backend.last_speech_score)));

    vad_status.values.push_back(
      make_key_value(
        "last_frame_clipping",
        bool_text(
          backend.last_frame_clipping)));

    vad_status.values.push_back(
      make_key_value(
        "backend_frames",
        std::to_string(
          backend.statistics.frames_analyzed)));

    vad_status.values.push_back(
      make_key_value(
        "samples_analyzed",
        std::to_string(
          backend.statistics.samples_analyzed)));

    vad_status.values.push_back(
      make_key_value(
        "noise_floor_updates",
        std::to_string(
          backend.statistics.noise_floor_updates)));

    vad_status.values.push_back(
      make_key_value(
        "noise_floor_freezes",
        std::to_string(
          backend.statistics.noise_floor_freezes)));

    vad_status.values.push_back(
      make_key_value(
        "clipping_frames",
        std::to_string(
          backend.statistics.clipping_frames)));

    vad_status.values.push_back(
      make_key_value(
        "backend_last_error",
        backend.last_error));

    vad_status.values.push_back(
      make_key_value(
        "state",
        std::string{
        vad::to_string(processor.state)}));

    vad_status.values.push_back(
      make_key_value(
        "processor_frames",
        std::to_string(
          processor.statistics.frames_processed)));

    vad_status.values.push_back(
      make_key_value(
        "speech_started_events",
        std::to_string(
          processor.statistics.speech_started_events)));

    vad_status.values.push_back(
      make_key_value(
        "speech_ended_events",
        std::to_string(
          processor.statistics.speech_ended_events)));

    vad_status.values.push_back(
      make_key_value(
        "active_segment_id",
        std::to_string(
          processor.active_segment_id)));

    vad_status.values.push_back(
      make_key_value(
        "queued_events",
        std::to_string(
          processor.queued_events)));

    vad_status.values.push_back(
      make_key_value(
        "events_dropped",
        std::to_string(
          processor.statistics.events_dropped)));

    vad_status.values.push_back(
      make_key_value(
        "processor_last_error",
        processor.last_error));
  }

  array.status.push_back(
    std::move(vad_status));

  diagnostic_msgs::msg::DiagnosticStatus
    utterance_status;

  utterance_status.name =
    "savo_speech/utterance_session";

  utterance_status.hardware_id =
    config_.device_id;

  utterance_status.values.push_back(
    make_key_value(
      "enabled",
      bool_text(
        config_.utterance_session_enabled)));

  utterance_status.values.push_back(
    make_key_value(
      "required",
      bool_text(
        config_.utterance_session_required)));

  utterance_status.values.push_back(
    make_key_value(
      "pre_roll_ms",
      std::to_string(
        config_.utterance_session_pre_roll_ms)));

  utterance_status.values.push_back(
    make_key_value(
      "speech_start_timeout_ms",
      std::to_string(
        config_.
        utterance_session_speech_start_timeout_ms)));

  utterance_status.values.push_back(
    make_key_value(
      "maximum_duration_ms",
      std::to_string(
        config_.
        utterance_session_maximum_duration_ms)));

  utterance_status.values.push_back(
    make_key_value(
      "completed_queue_capacity",
      std::to_string(
        config_.
        utterance_session_completed_queue_capacity)));

  utterance_status.values.push_back(
    make_key_value(
      "initialization_error",
      utterance_session_initialization_error_));

  if (!config_.utterance_session_enabled) {
    utterance_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::OK;

    utterance_status.message = "disabled";
  } else if (!utterance_session_processor_) {
    utterance_status.level =
      config_.utterance_session_required ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    utterance_status.message = "not_initialized";
  } else {
    const auto utterance =
      utterance_session_processor_->snapshot();

    const bool healthy =
      utterance.last_error.empty() &&
      utterance.session.last_error.empty();

    if (healthy) {
      utterance_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::OK;

      utterance_status.message =
        std::string{
        session::to_string(
          utterance.session.state)};
    } else {
      utterance_status.level =
        config_.utterance_session_required ?
        diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        diagnostic_msgs::msg::DiagnosticStatus::WARN;

      utterance_status.message = "degraded";
    }

    const std::string last_completion =
      utterance.session.last_completion_reason.has_value() ?
      std::string{
      session::to_string(
          *utterance.session.last_completion_reason)} :
    std::string{"none"};

    utterance_status.values.push_back(
      make_key_value(
        "state",
        std::string{
        session::to_string(
            utterance.session.state)}));

    utterance_status.values.push_back(
      make_key_value(
        "active_utterance_id",
        std::to_string(
          utterance.session.active_utterance_id)));

    utterance_status.values.push_back(
      make_key_value(
        "active_vad_segment_id",
        std::to_string(
          utterance.session.active_vad_segment_id)));

    utterance_status.values.push_back(
      make_key_value(
        "active_wake_phrase",
        utterance.session.active_wake_phrase));

    utterance_status.values.push_back(
      make_key_value(
        "active_audio_samples",
        std::to_string(
          utterance.session.active_audio_samples)));

    utterance_status.values.push_back(
      make_key_value(
        "queued_completed_utterances",
        std::to_string(
          utterance.session.
          queued_completed_utterances)));

    utterance_status.values.push_back(
      make_key_value(
        "pending_wake_events",
        std::to_string(
          utterance.pending_wake_events)));

    utterance_status.values.push_back(
      make_key_value(
        "pending_vad_events",
        std::to_string(
          utterance.pending_vad_events)));

    utterance_status.values.push_back(
      make_key_value(
        "processor_frames_received",
        std::to_string(
          utterance.statistics.frames_received)));

    utterance_status.values.push_back(
      make_key_value(
        "processor_frames_completed",
        std::to_string(
          utterance.statistics.frames_completed)));

    utterance_status.values.push_back(
      make_key_value(
        "processor_frames_failed",
        std::to_string(
          utterance.statistics.frames_failed)));

    utterance_status.values.push_back(
      make_key_value(
        "wake_events_accepted",
        std::to_string(
          utterance.statistics.
          wake_events_accepted)));

    utterance_status.values.push_back(
      make_key_value(
        "wake_events_rejected",
        std::to_string(
          utterance.statistics.
          wake_events_rejected)));

    utterance_status.values.push_back(
      make_key_value(
        "vad_events_accepted",
        std::to_string(
          utterance.statistics.
          vad_events_accepted)));

    utterance_status.values.push_back(
      make_key_value(
        "vad_events_rejected",
        std::to_string(
          utterance.statistics.
          vad_events_rejected)));

    utterance_status.values.push_back(
      make_key_value(
        "utterances_completed",
        std::to_string(
          utterance.session.statistics.
          utterances_completed)));

    utterance_status.values.push_back(
      make_key_value(
        "speech_ended_completions",
        std::to_string(
          utterance.session.statistics.
          speech_ended_completions)));

    utterance_status.values.push_back(
      make_key_value(
        "maximum_duration_completions",
        std::to_string(
          utterance.session.statistics.
          maximum_duration_completions)));

    utterance_status.values.push_back(
      make_key_value(
        "sessions_canceled",
        std::to_string(
          utterance.session.statistics.
          sessions_canceled)));

    utterance_status.values.push_back(
      make_key_value(
        "speech_start_timeouts",
        std::to_string(
          utterance.session.statistics.
          speech_start_timeouts)));

    utterance_status.values.push_back(
      make_key_value(
        "sequence_gaps",
        std::to_string(
          utterance.session.statistics.
          sequence_gaps)));

    utterance_status.values.push_back(
      make_key_value(
        "missing_audio_frames",
        std::to_string(
          utterance.session.statistics.
          missing_audio_frames)));

    utterance_status.values.push_back(
      make_key_value(
        "completed_queue_overflows",
        std::to_string(
          utterance.session.statistics.
          completed_queue_overflows)));

    utterance_status.values.push_back(
      make_key_value(
        "last_cancellation_reason",
        std::string{
        session::to_string(
            utterance.session.
          last_cancellation_reason)}));

    utterance_status.values.push_back(
      make_key_value(
        "last_completion_reason",
        last_completion));

    utterance_status.values.push_back(
      make_key_value(
        "processor_last_error",
        utterance.last_error));

    utterance_status.values.push_back(
      make_key_value(
        "session_last_error",
        utterance.session.last_error));
  }

  array.status.push_back(
    std::move(utterance_status));

  diagnostic_msgs::msg::DiagnosticStatus
    serialization_status;

  serialization_status.name =
    "savo_speech/utterance_serialization";

  serialization_status.hardware_id =
    config_.device_id;

  serialization_status.values.push_back(
    make_key_value(
      "enabled",
      bool_text(
        config_.utterance_serialization_enabled)));

  serialization_status.values.push_back(
    make_key_value(
      "required",
      bool_text(
        config_.utterance_serialization_required)));

  serialization_status.values.push_back(
    make_key_value(
      "output_queue_capacity",
      std::to_string(
        config_.
        utterance_serialization_output_queue_capacity)));

  serialization_status.values.push_back(
    make_key_value(
      "source_wait_timeout_ms",
      std::to_string(
        config_.
        utterance_serialization_source_wait_timeout_ms)));

  serialization_status.values.push_back(
    make_key_value(
      "maximum_wav_bytes",
      std::to_string(
        config_.
        utterance_serialization_maximum_wav_bytes)));

  serialization_status.values.push_back(
    make_key_value(
      "initialization_error",
      utterance_serialization_initialization_error_));

  if (!config_.utterance_serialization_enabled) {
    serialization_status.level =
      diagnostic_msgs::msg::DiagnosticStatus::OK;

    serialization_status.message = "disabled";
  } else if (!completed_utterance_worker_) {
    serialization_status.level =
      config_.utterance_serialization_required ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    serialization_status.message = "not_initialized";
  } else {
    const auto serialization =
      completed_utterance_worker_->snapshot();

    const std::string current_id =
      serialization.current_utterance_id.has_value() ?
      std::to_string(
        *serialization.current_utterance_id) :
      std::string{"none"};

    const std::string last_id =
      serialization.last_seen_utterance_id.has_value() ?
      std::to_string(
        *serialization.last_seen_utterance_id) :
      std::string{"none"};

    if (
      serialization.state ==
      session::CompletedUtteranceWorkerState::Faulted)
    {
      serialization_status.level =
        config_.utterance_serialization_required ?
        diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        diagnostic_msgs::msg::DiagnosticStatus::WARN;

      serialization_status.message = "faulted";
    } else if (
      serialization.state !=
      session::CompletedUtteranceWorkerState::Running)
    {
      serialization_status.level =
        config_.utterance_serialization_required ?
        diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        diagnostic_msgs::msg::DiagnosticStatus::WARN;

      serialization_status.message =
        std::string{
        session::to_string(serialization.state)};
    } else if (!serialization.last_error.empty()) {
      serialization_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::WARN;

      serialization_status.message = "degraded";
    } else {
      serialization_status.level =
        diagnostic_msgs::msg::DiagnosticStatus::OK;

      serialization_status.message = "running";
    }

    serialization_status.values.push_back(
      make_key_value(
        "state",
        std::string{
        session::to_string(serialization.state)}));

    serialization_status.values.push_back(
      make_key_value(
        "queue_size",
        std::to_string(
          serialization.output_queue.size)));

    serialization_status.values.push_back(
      make_key_value(
        "queue_capacity",
        std::to_string(
          serialization.output_queue.capacity)));

    serialization_status.values.push_back(
      make_key_value(
        "queue_accepted",
        std::to_string(
          serialization.output_queue.accepted)));

    serialization_status.values.push_back(
      make_key_value(
        "queue_rejected_invalid",
        std::to_string(
          serialization.output_queue.rejected_invalid)));

    serialization_status.values.push_back(
      make_key_value(
        "queue_rejected_duplicate_id",
        std::to_string(
          serialization.output_queue.
          rejected_duplicate_id)));

    serialization_status.values.push_back(
      make_key_value(
        "queue_rejected_full",
        std::to_string(
          serialization.output_queue.rejected_full)));

    serialization_status.values.push_back(
      make_key_value(
        "queue_popped",
        std::to_string(
          serialization.output_queue.popped)));

    serialization_status.values.push_back(
      make_key_value(
        "starts",
        std::to_string(
          serialization.statistics.starts)));

    serialization_status.values.push_back(
      make_key_value(
        "stops",
        std::to_string(
          serialization.statistics.stops)));

    serialization_status.values.push_back(
      make_key_value(
        "source_wait_timeouts",
        std::to_string(
          serialization.statistics.
          source_wait_timeouts)));

    serialization_status.values.push_back(
      make_key_value(
        "utterances_received",
        std::to_string(
          serialization.statistics.
          utterances_received)));

    serialization_status.values.push_back(
      make_key_value(
        "utterances_serialized",
        std::to_string(
          serialization.statistics.
          utterances_serialized)));

    serialization_status.values.push_back(
      make_key_value(
        "invalid_utterances",
        std::to_string(
          serialization.statistics.
          invalid_utterances)));

    serialization_status.values.push_back(
      make_key_value(
        "duplicate_or_out_of_order_ids",
        std::to_string(
          serialization.statistics.
          duplicate_or_out_of_order_ids)));

    serialization_status.values.push_back(
      make_key_value(
        "encoding_failures",
        std::to_string(
          serialization.statistics.
          encoding_failures)));

    serialization_status.values.push_back(
      make_key_value(
        "wav_size_limit_rejections",
        std::to_string(
          serialization.statistics.
          wav_size_limit_rejections)));

    serialization_status.values.push_back(
      make_key_value(
        "output_queue_rejections",
        std::to_string(
          serialization.statistics.
          output_queue_rejections)));

    serialization_status.values.push_back(
      make_key_value(
        "faults",
        std::to_string(
          serialization.statistics.faults)));

    serialization_status.values.push_back(
      make_key_value(
        "current_utterance_id",
        current_id));

    serialization_status.values.push_back(
      make_key_value(
        "last_seen_utterance_id",
        last_id));

    serialization_status.values.push_back(
      make_key_value(
        "last_error",
        serialization.last_error));
  }

  array.status.push_back(
    std::move(serialization_status));

  return array;
}

}  // namespace savo_speech
