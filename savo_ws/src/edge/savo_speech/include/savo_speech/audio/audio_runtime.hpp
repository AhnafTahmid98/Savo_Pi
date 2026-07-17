#ifndef SAVO_SPEECH__AUDIO__AUDIO_RUNTIME_HPP_
#define SAVO_SPEECH__AUDIO__AUDIO_RUNTIME_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include "savo_speech/audio/audio_frame.hpp"
#include "savo_speech/audio/capture_pipeline.hpp"
#include "savo_speech/audio/capture_source.hpp"
#include "savo_speech/audio/capture_worker.hpp"
#include "savo_speech/audio/captured_frame_source.hpp"
#include "savo_speech/audio/microphone_gate.hpp"
#include "savo_speech/audio/playback_sink.hpp"
#include "savo_speech/audio/playback_worker.hpp"

namespace savo_speech::audio
{

enum class AudioRuntimeState : std::uint8_t
{
  Stopped = 0U,
  Starting = 1U,
  Running = 2U,
  Stopping = 3U,
  Faulted = 4U
};

[[nodiscard]] constexpr std::string_view to_string(
  const AudioRuntimeState state) noexcept
{
  switch (state) {
    case AudioRuntimeState::Stopped:
      return "stopped";

    case AudioRuntimeState::Starting:
      return "starting";

    case AudioRuntimeState::Running:
      return "running";

    case AudioRuntimeState::Stopping:
      return "stopping";

    case AudioRuntimeState::Faulted:
      return "faulted";
  }

  return "unknown";
}

struct AudioRuntimeConfig
{
  MicrophoneGateConfig microphone_gate{};
  CapturePipelineConfig capture_pipeline{};
  PlaybackWorkerConfig playback_worker{};

  [[nodiscard]] bool is_valid() const noexcept
  {
    return
      microphone_gate.is_valid() &&
      capture_pipeline.is_valid() &&
      playback_worker.is_valid();
  }
};

struct AudioRuntimeStatistics
{
  std::uint64_t starts{0U};
  std::uint64_t stops{0U};

  std::uint64_t start_failures{0U};
  std::uint64_t startup_rollbacks{0U};

  std::uint64_t playback_enqueue_attempts{0U};
  std::uint64_t playback_enqueue_accepted{0U};
  std::uint64_t playback_enqueue_rejected{0U};
};

struct AudioRuntimeHealthSnapshot
{
  AudioRuntimeState state{
    AudioRuntimeState::Stopped};

  bool ready{false};

  CaptureWorkerState capture_worker_state{
    CaptureWorkerState::Stopped};

  PlaybackWorkerState playback_worker_state{
    PlaybackWorkerState::Stopped};

  MicrophoneGateState microphone_gate{};

  CaptureWorkerStatistics capture_worker_statistics{};
  CapturePipelineStatistics capture_pipeline_statistics{};

  BoundedAudioQueueStatistics capture_queue_statistics{};
  AudioRingBufferStatistics pre_roll_statistics{};

  PlaybackWorkerStatistics playback_worker_statistics{};
  AudioRuntimeStatistics runtime_statistics{};

  std::size_t pending_playback_requests{0U};

  std::optional<std::uint64_t>
  current_playback_request_id{};

  std::string last_error{};
};

class AudioRuntime final : public CapturedFrameSource
{
public:
  AudioRuntime(
    CaptureSource & capture_source,
    PlaybackSink & playback_sink,
    AudioRuntimeConfig config = AudioRuntimeConfig{});

  ~AudioRuntime();

  AudioRuntime(const AudioRuntime &) = delete;
  AudioRuntime & operator=(const AudioRuntime &) = delete;

  AudioRuntime(AudioRuntime &&) = delete;
  AudioRuntime & operator=(AudioRuntime &&) = delete;

  [[nodiscard]] bool start();

  void stop() noexcept;

  [[nodiscard]] PlaybackEnqueueResult enqueue_playback(
    PlaybackRequest request);

  [[nodiscard]] bool cancel_current_playback() noexcept;

  [[nodiscard]] std::size_t cancel_pending_playback();

  void cancel_all_playback();

  [[nodiscard]] std::optional<PlaybackCompletion>
  try_pop_playback_completion();

  [[nodiscard]] std::optional<PlaybackCompletion>
  wait_playback_completion_for(
    std::chrono::milliseconds timeout);

  [[nodiscard]] std::optional<AudioFrame>
  try_pop_captured_frame();

  [[nodiscard]] std::optional<AudioFrame>
  wait_captured_frame_for(
    std::chrono::milliseconds timeout) override;

  [[nodiscard]] AudioBuffer pre_roll_snapshot() const;

  [[nodiscard]] AudioRuntimeState state() const noexcept;
  [[nodiscard]] bool ready() const noexcept;

  [[nodiscard]] AudioRuntimeStatistics
  statistics() const noexcept;

  [[nodiscard]] AudioRuntimeHealthSnapshot
  health_snapshot() const;

  [[nodiscard]] std::string last_error() const;

private:
  void record_start_failure(
    const std::string & error,
    bool playback_was_started) noexcept;

  CaptureSource & capture_source_;
  PlaybackSink & playback_sink_;

  AudioRuntimeConfig config_;

  MicrophoneGate microphone_gate_;
  CapturePipeline capture_pipeline_;

  CaptureWorker capture_worker_;
  PlaybackWorker playback_worker_;

  mutable std::mutex mutex_;

  AudioRuntimeState state_{
    AudioRuntimeState::Stopped};

  AudioRuntimeStatistics statistics_{};

  std::string last_error_{};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__AUDIO_RUNTIME_HPP_
