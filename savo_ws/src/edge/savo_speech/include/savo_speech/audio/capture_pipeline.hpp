#ifndef SAVO_SPEECH__AUDIO__CAPTURE_PIPELINE_HPP_
#define SAVO_SPEECH__AUDIO__CAPTURE_PIPELINE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/audio_frame.hpp"
#include "savo_speech/audio/audio_ring_buffer.hpp"
#include "savo_speech/audio/bounded_audio_queue.hpp"
#include "savo_speech/audio/microphone_gate.hpp"

namespace savo_speech::audio
{

struct CapturePipelineConfig
{
  std::uint16_t selected_channel{0U};

  std::size_t pre_roll_samples{16000U};
  std::size_t queue_capacity_frames{50U};

  QueueOverflowPolicy queue_overflow_policy{
    QueueOverflowPolicy::DropOldest};

  [[nodiscard]] bool is_valid() const noexcept
  {
    return
      selected_channel < 32U &&
      pre_roll_samples > 0U &&
      queue_capacity_frames > 0U &&
      queue_capacity_frames <= 10000U;
  }
};

enum class CapturePipelineResult : std::uint8_t
{
  Accepted = 0U,
  AcceptedAfterDroppingOldest = 1U,
  DroppedWhileGated = 2U,
  RejectedInvalidFrame = 3U,
  RejectedChannelUnavailable = 4U,
  RejectedQueueFull = 5U,
  RejectedQueueClosed = 6U
};

struct CapturePipelineStatistics
{
  std::uint64_t received_frames{0U};
  std::uint64_t accepted_frames{0U};

  std::uint64_t gated_frames{0U};
  std::uint64_t invalid_frames{0U};
  std::uint64_t channel_errors{0U};

  std::uint64_t queue_drop_events{0U};
  std::uint64_t queue_rejections{0U};

  std::uint64_t gate_flushes{0U};
};

class CapturePipeline final
{
public:
  using Clock = MicrophoneGate::Clock;

  CapturePipeline(
    CapturePipelineConfig config,
    MicrophoneGate & microphone_gate);

  CapturePipeline(const CapturePipeline &) = delete;
  CapturePipeline & operator=(const CapturePipeline &) = delete;

  CapturePipeline(CapturePipeline &&) = delete;
  CapturePipeline & operator=(CapturePipeline &&) = delete;

  [[nodiscard]] CapturePipelineResult process(
    AudioFrame frame,
    Clock::time_point now = Clock::now());

  [[nodiscard]] std::optional<AudioFrame> try_pop();

  [[nodiscard]] std::optional<AudioFrame> wait_pop_for(
    std::chrono::milliseconds timeout);

  [[nodiscard]] AudioBuffer pre_roll_snapshot() const;

  void clear() noexcept;
  void close() noexcept;

  [[nodiscard]] CapturePipelineStatistics statistics() const noexcept;

  [[nodiscard]] BoundedAudioQueueStatistics
  queue_statistics() const noexcept;

  [[nodiscard]] AudioRingBufferStatistics
  pre_roll_statistics() const noexcept;

private:
  void enter_gated_state();

  CapturePipelineConfig config_;
  MicrophoneGate & microphone_gate_;

  AudioRingBuffer pre_roll_;
  BoundedAudioQueue queue_;

  mutable std::mutex state_mutex_;

  bool previously_gated_{false};

  std::uint32_t output_sample_rate_hz_{0U};

  CapturePipelineStatistics statistics_{};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__CAPTURE_PIPELINE_HPP_
