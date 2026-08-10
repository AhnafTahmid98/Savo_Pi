// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__SESSION__UTTERANCE_SESSION_CORE_HPP_
#define SAVO_SPEECH__SESSION__UTTERANCE_SESSION_CORE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "savo_speech/session/utterance_session_event.hpp"

#include "savo_speech/audio/audio_format.hpp"
#include "savo_speech/audio/audio_frame.hpp"
#include "savo_speech/audio/audio_ring_buffer.hpp"
#include "savo_speech/session/completed_utterance.hpp"
#include "savo_speech/vad/vad_event.hpp"
#include "savo_speech/wake_word/wake_word_event.hpp"

namespace savo_speech::session
{

struct UtteranceSessionConfig
{
  std::chrono::milliseconds pre_roll_duration{1000};
  std::chrono::milliseconds speech_start_timeout{3000};
  std::chrono::milliseconds minimum_speech_duration{300};
  std::chrono::milliseconds maximum_utterance_duration{15000};

  std::size_t completed_queue_capacity{4U};

  [[nodiscard]] bool is_valid() const noexcept;
};

struct UtteranceSessionStatistics
{
  std::uint64_t audio_frames_processed{0U};
  std::uint64_t audio_samples_processed{0U};

  std::uint64_t duplicate_audio_frames{0U};
  std::uint64_t out_of_order_audio_frames{0U};
  std::uint64_t out_of_order_audio_timestamps{0U};
  std::uint64_t audio_format_mismatches{0U};

  std::uint64_t sequence_gaps{0U};
  std::uint64_t missing_audio_frames{0U};

  std::uint64_t wake_events_received{0U};
  std::uint64_t wake_events_accepted{0U};
  std::uint64_t stale_wake_events{0U};
  std::uint64_t wake_events_ignored_while_active{0U};

  std::uint64_t vad_events_received{0U};
  std::uint64_t vad_events_accepted{0U};
  std::uint64_t stale_vad_events{0U};
  std::uint64_t invalid_vad_transitions{0U};

  std::uint64_t sessions_armed{0U};
  std::uint64_t recordings_started{0U};

  std::uint64_t utterances_completed{0U};
  std::uint64_t speech_ended_completions{0U};
  std::uint64_t maximum_duration_completions{0U};
  std::uint64_t short_speech_rejections{0U};

  std::uint64_t sessions_canceled{0U};
  std::uint64_t speech_start_timeouts{0U};
  std::uint64_t explicit_cancellations{0U};
  std::uint64_t no_audio_cancellations{0U};

  std::uint64_t completed_queue_overflows{0U};
  std::uint64_t completed_utterances_dropped{0U};

  std::uint64_t last_audio_frame_sequence{0U};
  std::uint64_t last_wake_event_id{0U};
  std::uint64_t last_vad_event_id{0U};
  std::uint64_t last_utterance_id{0U};
};

struct UtteranceSessionSnapshot
{
  UtteranceSessionState state{
    UtteranceSessionState::Idle};

  UtteranceSessionStatistics statistics{};

  bool audio_format_initialized{false};

  std::size_t pre_roll_samples{0U};
  std::size_t active_audio_samples{0U};
  std::size_t queued_completed_utterances{0U};

  std::uint64_t active_utterance_id{0U};
  std::uint64_t active_vad_segment_id{0U};

  std::string active_wake_phrase{};

  bool pending_speech_end{false};

  UtteranceCancellationReason last_cancellation_reason{
    UtteranceCancellationReason::None};

  std::optional<UtteranceCompletionReason>
  last_completion_reason{};

  std::string last_error{};
};

class UtteranceSessionCore final
{
public:
  using Clock = audio::AudioFrame::Clock;

  explicit UtteranceSessionCore(
    UtteranceSessionConfig config =
    UtteranceSessionConfig{});

  ~UtteranceSessionCore() = default;

  UtteranceSessionCore(
    const UtteranceSessionCore &) = delete;

  UtteranceSessionCore & operator=(
    const UtteranceSessionCore &) = delete;

  UtteranceSessionCore(
    UtteranceSessionCore &&) = delete;

  UtteranceSessionCore & operator=(
    UtteranceSessionCore &&) = delete;

  void process_audio_frame(
    const audio::AudioFrame & frame);

  [[nodiscard]] bool handle_wake_word_event(
    const wake_word::WakeWordEvent & event);

  [[nodiscard]] bool handle_vad_event(
    const vad::VadEvent & event);

  [[nodiscard]] bool begin_follow_up(
    Clock::time_point now);

  void advance_time(
    Clock::time_point now);

  [[nodiscard]] bool cancel(
    UtteranceCancellationReason reason =
    UtteranceCancellationReason::
    ExplicitCancellation);

  [[nodiscard]] std::optional<CompletedUtterance>
  try_pop_completed();

  [[nodiscard]] std::optional<CompletedUtterance>
  wait_completed_for(
    std::chrono::milliseconds timeout);

  [[nodiscard]] UtteranceSessionSnapshot
  snapshot() const;

  void reset() noexcept;

private:
  struct FrameSpan
  {
    std::uint64_t sequence{0U};
    std::size_t sample_count{0U};
    Clock::time_point captured_at{};
  };

  void initialize_audio_format_locked(
    const audio::AudioFormat & format);

  [[nodiscard]] bool same_audio_format_locked(
    const audio::AudioFormat & format) const noexcept;

  void append_pre_roll_locked(
    const audio::AudioFrame & frame);

  void start_recording_locked(
    const vad::VadEvent & event);

  void append_active_frame_locked(
    const audio::AudioFrame & frame);

  void trim_active_after_sequence_locked(
    std::uint64_t sequence);

  [[nodiscard]] bool finalize_locked(
    UtteranceCompletionReason reason,
    Clock::time_point completed_at,
    std::uint64_t speech_end_frame_sequence);

  void reject_short_utterance_locked(
    std::uint64_t speech_end_frame_sequence);

  [[nodiscard]] bool check_timeouts_locked(
    Clock::time_point now);

  [[nodiscard]] bool cancel_locked(
    UtteranceCancellationReason reason);

  void clear_active_utterance_locked() noexcept;
  void clear_conversation_locked() noexcept;

  void enqueue_completed_locked(
    CompletedUtterance utterance);

  [[nodiscard]] std::pair<std::uint64_t, std::uint64_t>
  active_gap_metadata_locked() const noexcept;

  UtteranceSessionConfig config_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;

  UtteranceSessionState state_{
    UtteranceSessionState::Idle};

  UtteranceSessionStatistics statistics_{};

  std::optional<audio::AudioFormat> audio_format_{};

  std::unique_ptr<audio::AudioRingBuffer>
  pre_roll_buffer_;

  std::deque<FrameSpan> pre_roll_spans_{};

  std::optional<std::uint64_t>
  last_audio_sequence_{};

  std::optional<Clock::time_point>
  last_audio_captured_at_{};

  std::optional<Clock::time_point>
  last_advanced_time_{};

  // The listening boundary is independent from the original
  // wake event. Initial listening begins at wake detection;
  // follow-up listening begins only after playback completes.
  std::optional<std::uint64_t>
  listening_boundary_sequence_{};

  std::optional<Clock::time_point>
  listening_started_at_{};

  std::optional<wake_word::WakeWordEvent>
  active_wake_event_{};

  std::optional<vad::VadEvent>
  active_speech_start_{};

  std::optional<vad::VadEvent>
  pending_speech_end_{};

  std::uint64_t active_utterance_id_{0U};

  std::vector<std::int16_t> active_samples_{};
  std::deque<FrameSpan> active_spans_{};

  std::size_t active_pre_roll_samples_{0U};

  std::deque<CompletedUtterance>
  completed_utterances_{};

  UtteranceCancellationReason
    last_cancellation_reason_{
    UtteranceCancellationReason::None};

  std::optional<UtteranceCompletionReason>
  last_completion_reason_{};

  std::string last_error_{};
};

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__UTTERANCE_SESSION_CORE_HPP_
