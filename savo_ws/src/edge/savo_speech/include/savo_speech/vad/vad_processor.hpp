// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__VAD__VAD_PROCESSOR_HPP_
#define SAVO_SPEECH__VAD__VAD_PROCESSOR_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include "savo_speech/vad/vad_backend.hpp"
#include "savo_speech/vad/vad_event.hpp"

#include "savo_speech/audio/captured_audio_processor.hpp"

namespace savo_speech::vad
{

struct VadProcessorConfig
{
  double speech_start_threshold{0.65};
  double speech_end_threshold{0.35};

  std::size_t required_start_frames{3U};
  std::size_t required_end_frames{10U};

  std::size_t event_queue_capacity{8U};

  [[nodiscard]] bool is_valid() const noexcept;
};

struct VadProcessorStatistics
{
  std::uint64_t frames_processed{0U};

  std::uint64_t start_candidate_frames{0U};
  std::uint64_t end_candidate_frames{0U};

  std::uint64_t start_debounce_resets{0U};
  std::uint64_t end_debounce_resets{0U};

  std::uint64_t speech_started_events{0U};
  std::uint64_t speech_ended_events{0U};

  std::uint64_t queue_overflows{0U};
  std::uint64_t events_dropped{0U};

  std::uint64_t backend_failures{0U};
  std::uint64_t invalid_backend_results{0U};

  std::uint64_t last_frame_sequence{0U};
  std::uint64_t last_event_id{0U};
  std::uint64_t last_segment_id{0U};
};

struct VadProcessorSnapshot
{
  VadState state{VadState::Silence};

  VadProcessorStatistics statistics{};

  std::size_t queued_events{0U};

  std::size_t consecutive_start_frames{0U};
  std::size_t consecutive_end_frames{0U};

  std::uint64_t active_segment_id{0U};

  double last_speech_score{0.0};

  std::optional<VadEvent> last_event{};

  std::string last_error{};
};

class VadProcessor final
  : public audio::CapturedAudioProcessor
{
public:
  VadProcessor(
    VadBackend & backend,
    VadProcessorConfig config =
    VadProcessorConfig{});

  ~VadProcessor() override = default;

  VadProcessor(const VadProcessor &) = delete;
  VadProcessor & operator=(const VadProcessor &) = delete;

  VadProcessor(VadProcessor &&) = delete;
  VadProcessor & operator=(VadProcessor &&) = delete;

  void process(
    const audio::AudioFrame & frame) override;

  [[nodiscard]] std::optional<VadEvent>
  try_pop_event();

  [[nodiscard]] std::optional<VadEvent>
  wait_event_for(
    std::chrono::milliseconds timeout);

  [[nodiscard]] VadProcessorSnapshot
  snapshot() const;

  void reset() noexcept;

private:
  void reset_debounce_locked() noexcept;

  void enqueue_transition_locked(
    VadEventType type,
    const audio::AudioFrame & frame,
    double speech_score);

  void enqueue_event_locked(
    VadEvent event);

  VadBackend & backend_;
  VadProcessorConfig config_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;

  std::deque<VadEvent> events_{};

  VadState state_{VadState::Silence};

  VadProcessorStatistics statistics_{};

  std::size_t consecutive_start_frames_{0U};
  std::size_t consecutive_end_frames_{0U};

  std::uint64_t active_segment_id_{0U};

  double last_speech_score_{0.0};

  std::optional<VadEvent> last_event_{};

  std::string last_error_{};
};

}  // namespace savo_speech::vad

#endif  // SAVO_SPEECH__VAD__VAD_PROCESSOR_HPP_
