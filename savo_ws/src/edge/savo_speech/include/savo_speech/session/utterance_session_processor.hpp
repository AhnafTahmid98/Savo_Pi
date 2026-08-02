// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__SESSION__UTTERANCE_SESSION_PROCESSOR_HPP_
#define SAVO_SPEECH__SESSION__UTTERANCE_SESSION_PROCESSOR_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "savo_speech/session/utterance_session_core.hpp"
#include "savo_speech/session/utterance_session_event.hpp"

#include "savo_speech/audio/captured_audio_processor.hpp"
#include "savo_speech/session/completed_utterance.hpp"
#include "savo_speech/session/completed_utterance_source.hpp"
#include "savo_speech/vad/vad_processor.hpp"
#include "savo_speech/wake_word/wake_word_processor.hpp"

namespace savo_speech::session
{

struct UtteranceSessionProcessorStatistics
{
  std::uint64_t frames_received{0U};
  std::uint64_t frames_completed{0U};
  std::uint64_t frames_failed{0U};

  std::uint64_t wake_events_drained{0U};
  std::uint64_t wake_events_accepted{0U};
  std::uint64_t wake_events_rejected{0U};

  std::uint64_t vad_events_drained{0U};
  std::uint64_t vad_events_accepted{0U};
  std::uint64_t vad_events_rejected{0U};

  std::uint64_t last_frame_sequence{0U};
};

struct UtteranceSessionProcessorSnapshot
{
  UtteranceSessionProcessorStatistics statistics{};

  UtteranceSessionSnapshot session{};

  std::size_t pending_wake_events{0U};
  std::size_t pending_vad_events{0U};

  std::string last_error{};
};

class UtteranceSessionProcessor final
  : public audio::CapturedAudioProcessor,
  public CompletedUtteranceSource
{
public:
  UtteranceSessionProcessor(
    wake_word::WakeWordProcessor & wake_word_processor,
    vad::VadProcessor & vad_processor,
    UtteranceSessionConfig session_config =
    UtteranceSessionConfig{});

  ~UtteranceSessionProcessor() override = default;

  UtteranceSessionProcessor(
    const UtteranceSessionProcessor &) = delete;

  UtteranceSessionProcessor & operator=(
    const UtteranceSessionProcessor &) = delete;

  UtteranceSessionProcessor(
    UtteranceSessionProcessor &&) = delete;

  UtteranceSessionProcessor & operator=(
    UtteranceSessionProcessor &&) = delete;

  void process(
    const audio::AudioFrame & frame) override;

  [[nodiscard]] bool cancel(
    UtteranceCancellationReason reason =
    UtteranceCancellationReason::
    ExplicitCancellation);

  [[nodiscard]] std::optional<CompletedUtterance>
  try_pop_completed();

  [[nodiscard]] std::optional<CompletedUtterance>
  wait_completed_for(
    std::chrono::milliseconds timeout) override;

  [[nodiscard]] UtteranceSessionProcessorSnapshot
  snapshot() const;

  void reset() noexcept;

private:
  wake_word::WakeWordProcessor &
  wake_word_processor_;

  vad::VadProcessor &
  vad_processor_;

  UtteranceSessionCore session_core_;

  mutable std::mutex operation_mutex_;
  mutable std::mutex statistics_mutex_;

  UtteranceSessionProcessorStatistics statistics_{};

  std::string last_error_{};
};

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__UTTERANCE_SESSION_PROCESSOR_HPP_
