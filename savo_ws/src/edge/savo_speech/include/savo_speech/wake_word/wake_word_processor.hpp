#ifndef SAVO_SPEECH__WAKE_WORD__WAKE_WORD_PROCESSOR_HPP_
#define SAVO_SPEECH__WAKE_WORD__WAKE_WORD_PROCESSOR_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include "savo_speech/audio/captured_audio_processor.hpp"
#include "savo_speech/wake_word/wake_word_backend.hpp"
#include "savo_speech/wake_word/wake_word_event.hpp"

namespace savo_speech::wake_word
{

struct WakeWordProcessorConfig
{
  double confidence_threshold{0.65};

  std::size_t required_consecutive_detections{1U};
  std::chrono::milliseconds cooldown{2000};

  std::size_t event_queue_capacity{8U};

  [[nodiscard]] bool is_valid() const noexcept;
};

struct WakeWordProcessorStatistics
{
  std::uint64_t frames_processed{0U};

  std::uint64_t backend_detections{0U};
  std::uint64_t accepted_detections{0U};

  std::uint64_t below_threshold{0U};
  std::uint64_t debounce_resets{0U};
  std::uint64_t cooldown_suppressed{0U};

  std::uint64_t queue_overflows{0U};
  std::uint64_t events_dropped{0U};

  std::uint64_t backend_failures{0U};

  std::uint64_t last_frame_sequence{0U};
  std::uint64_t last_event_id{0U};
};

struct WakeWordProcessorSnapshot
{
  WakeWordProcessorStatistics statistics{};

  std::size_t queued_events{0U};
  std::size_t consecutive_detections{0U};

  std::string candidate_phrase{};
  std::string last_detected_phrase{};

  double candidate_confidence{0.0};
  double last_detected_confidence{0.0};

  std::string last_error{};
};

class WakeWordProcessor final :
  public audio::CapturedAudioProcessor
{
public:
  WakeWordProcessor(
    WakeWordBackend & backend,
    WakeWordProcessorConfig config =
      WakeWordProcessorConfig{});

  ~WakeWordProcessor() override = default;

  WakeWordProcessor(
    const WakeWordProcessor &) = delete;

  WakeWordProcessor & operator=(
    const WakeWordProcessor &) = delete;

  WakeWordProcessor(
    WakeWordProcessor &&) = delete;

  WakeWordProcessor & operator=(
    WakeWordProcessor &&) = delete;

  void process(
    const audio::AudioFrame & frame) override;

  [[nodiscard]] std::optional<WakeWordEvent>
  try_pop_event();

  [[nodiscard]] std::optional<WakeWordEvent>
  wait_event_for(
    std::chrono::milliseconds timeout);

  [[nodiscard]] WakeWordProcessorSnapshot
  snapshot() const;

  void reset() noexcept;

private:
  void reset_candidate_locked() noexcept;

  void enqueue_event_locked(
    WakeWordEvent event);

  WakeWordBackend & backend_;
  WakeWordProcessorConfig config_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;

  std::deque<WakeWordEvent> events_{};

  WakeWordProcessorStatistics statistics_{};

  std::size_t consecutive_detections_{0U};

  std::string candidate_phrase_{};
  double candidate_confidence_{0.0};

  std::optional<WakeWordEvent::Clock::time_point>
  last_accepted_at_{};

  std::string last_detected_phrase_{};
  double last_detected_confidence_{0.0};

  std::string last_error_{};
};

}  // namespace savo_speech::wake_word

#endif  // SAVO_SPEECH__WAKE_WORD__WAKE_WORD_PROCESSOR_HPP_
