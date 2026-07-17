#include "savo_speech/speech_node.hpp"

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

  capture_processing_dispatcher_.reset();
  captured_audio_processor_chain_.reset();
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
          phase_ = session::SpeechPhase::Idle;
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

  return array;
}

}  // namespace savo_speech
