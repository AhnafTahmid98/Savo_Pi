// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__SPEECH_NODE_HPP_
#define SAVO_SPEECH__SPEECH_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "savo_speech/session/speech_error.hpp"
#include "savo_speech/session/speech_phase.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "savo_speech/audio/audio_activity_monitor.hpp"
#include "savo_speech/audio/audio_runtime.hpp"
#include "savo_speech/audio/capture_processing_dispatcher.hpp"
#include "savo_speech/audio/captured_audio_processor_chain.hpp"
#include "savo_speech/drivers/alsa_capture_stream.hpp"
#include "savo_speech/drivers/alsa_playback_stream.hpp"
#include "savo_speech/session/completed_utterance_worker.hpp"
#include "savo_speech/session/utterance_session_processor.hpp"
#include "savo_speech/transport/savomind_round_trip_worker.hpp"
#include "savo_speech/transport/savomind_transport.hpp"
#include "savo_speech/vad/adaptive_energy_vad_backend.hpp"
#include "savo_speech/vad/vad_processor.hpp"
#include "savo_speech/wake_word/pocketsphinx_wake_word_backend.hpp"
#include "savo_speech/wake_word/wake_word_processor.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace savo_speech
{

class SpeechNode final : public rclcpp::Node
{
public:
  explicit SpeechNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~SpeechNode() override;

private:
  struct RuntimeConfig
  {
    bool enabled{true};

    std::string profile{"edge_real_robot_v1"};
    std::string robot_id{"robot-savo"};
    std::string host_role{"edge"};
    std::string device_id{"savo-edge"};

    bool audio_required{true};

    std::string capture_device{"savo_respeaker"};
    std::string playback_device{"savo_respeaker"};

    bool allow_numeric_device_fallback{false};
    bool require_exact_sample_rate{true};

    std::int64_t sample_rate_hz{16000};

    std::int64_t capture_channels{6};
    std::int64_t playback_channels{1};
    std::int64_t selected_channel{0};

    std::int64_t period_frames{320};
    std::int64_t periods{4};
    std::int64_t chunk_frames{320};

    std::int64_t pre_roll_ms{1000};
    std::int64_t post_playback_hold_ms{250};

    std::int64_t capture_queue_capacity_frames{100};
    std::int64_t playback_queue_capacity{8};

    std::int64_t processing_wait_timeout_ms{100};
    std::int64_t processing_freshness_timeout_ms{1000};

    bool processing_fault_on_processor_error{true};

    bool wake_word_enabled{false};
    bool wake_word_required{true};

    std::string wake_word_profile{"default"};

    std::string wake_word_acoustic_model_path{
      "/usr/share/pocketsphinx/model/en-us/en-us"};

    std::string wake_word_dictionary_path{};
    std::string wake_word_keyword_file_path{};

    std::string wake_word_search_name{
      "savo_wake_words"};

    bool wake_word_suppress_decoder_log{true};

    double wake_word_confidence_threshold{0.65};

    std::int64_t wake_word_consecutive_detections{1};
    std::int64_t wake_word_cooldown_ms{2000};
    std::int64_t wake_word_event_queue_capacity{8};

    bool vad_enabled{false};
    bool vad_required{true};

    std::int64_t vad_startup_calibration_frames{25};

    double vad_initial_noise_floor_rms{0.005};
    double vad_minimum_noise_floor_rms{0.0005};
    double vad_maximum_noise_floor_rms{0.250};

    double vad_noise_floor_update_alpha{0.05};
    double vad_minimum_speech_rms{0.010};

    double vad_speech_onset_snr_db{8.0};
    double vad_speech_saturation_snr_db{20.0};

    std::int64_t vad_clipping_threshold{32760};

    double vad_speech_start_threshold{0.65};
    double vad_speech_end_threshold{0.35};

    std::int64_t vad_required_start_frames{3};
    std::int64_t vad_required_end_frames{10};
    std::int64_t vad_event_queue_capacity{8};

    bool utterance_session_enabled{false};
    bool utterance_session_required{true};

    std::int64_t utterance_session_pre_roll_ms{1000};

    std::int64_t
      utterance_session_speech_start_timeout_ms{3000};

    std::int64_t
      utterance_session_minimum_speech_duration_ms{300};

    std::int64_t
      utterance_session_maximum_duration_ms{15000};

    std::int64_t
      utterance_session_completed_queue_capacity{4};

    bool utterance_serialization_enabled{false};
    bool utterance_serialization_required{true};

    std::int64_t
      utterance_serialization_output_queue_capacity{4};

    std::int64_t
      utterance_serialization_source_wait_timeout_ms{100};

    std::int64_t
      utterance_serialization_maximum_wav_bytes{
      2 * 1024 * 1024};

    bool savomind_enabled{true};
    bool savomind_required{true};
    std::string savomind_socket_path{"/run/savomind/speech.sock"};
    std::int64_t savomind_connect_timeout_ms{1500};
    std::int64_t savomind_io_timeout_ms{30000};
    std::int64_t savomind_source_wait_timeout_ms{100};
    std::int64_t savomind_playback_timeout_ms{60000};
    std::int64_t savomind_maximum_request_bytes{2 * 1024 * 1024};
    std::int64_t savomind_maximum_response_bytes{16 * 1024 * 1024};
    std::int64_t savomind_maximum_text_bytes{8192};
    bool savomind_require_server_uid{false};
    std::int64_t savomind_server_uid{10001};

    double status_publish_rate_hz{2.0};
    double heartbeat_rate_hz{1.0};
  };

  void declare_parameters();
  void load_parameters();
  void validate_parameters() const;

  void configure_initial_state();

  void initialize_audio_runtime();
  void shutdown_audio_runtime() noexcept;
  void initialize_savomind_runtime();
  void shutdown_savomind_runtime() noexcept;
  void enqueue_savomind_event(const transport::RoundTripEvent & event);
  void process_savomind_events();
  void refresh_runtime_state();

  void publish_runtime_status();
  void publish_speech_state();
  void publish_heartbeat();

  [[nodiscard]] std::string readiness_text() const;
  [[nodiscard]] std::string dashboard_text() const;

  [[nodiscard]]
  diagnostic_msgs::msg::DiagnosticArray create_diagnostics() const;

  RuntimeConfig config_{};

  session::SpeechPhase phase_{
    session::SpeechPhase::Starting};

  session::SpeechError error_{
    session::SpeechError::None};

  bool ready_{false};
  bool audio_initialized_{false};
  bool savomind_initialized_{false};

  std::string reason_{"starting"};

  std::string wake_word_initialization_error_{};
  std::string vad_initialization_error_{};
  std::string utterance_session_initialization_error_{};
  std::string utterance_serialization_initialization_error_{};

  std::uint64_t heartbeat_count_{0U};

  std::unique_ptr<drivers::AlsaCaptureStream>
  capture_stream_;

  std::unique_ptr<drivers::AlsaPlaybackStream>
  playback_stream_;

  std::unique_ptr<audio::AudioRuntime>
  audio_runtime_;

  std::unique_ptr<audio::AudioActivityMonitor>
  audio_activity_monitor_;

  std::unique_ptr<
    wake_word::PocketSphinxWakeWordBackend>
  wake_word_backend_;

  std::unique_ptr<wake_word::WakeWordProcessor>
  wake_word_processor_;

  std::unique_ptr<vad::AdaptiveEnergyVadBackend>
  vad_backend_;

  std::unique_ptr<vad::VadProcessor>
  vad_processor_;

  std::unique_ptr<
    session::UtteranceSessionProcessor>
  utterance_session_processor_;

  std::unique_ptr<
    session::CompletedUtteranceWorker>
  completed_utterance_worker_;

  std::unique_ptr<transport::UnixSocketSavoMindTransport>
  savomind_transport_;

  std::unique_ptr<transport::SavoMindRoundTripWorker>
  savomind_round_trip_worker_;

  mutable std::mutex savomind_event_mutex_;
  std::deque<transport::RoundTripEvent> savomind_events_;
  transport::RoundTripSnapshot savomind_snapshot_{};

  std::unique_ptr<audio::CapturedAudioProcessorChain>
  captured_audio_processor_chain_;

  std::unique_ptr<audio::CaptureProcessingDispatcher>
  capture_processing_dispatcher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    readiness_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    dashboard_publisher_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr
    heartbeat_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    transcript_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    response_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    playback_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    playback_finished_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    result_publisher_;

  rclcpp::Publisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace savo_speech

#endif  // SAVO_SPEECH__SPEECH_NODE_HPP_
