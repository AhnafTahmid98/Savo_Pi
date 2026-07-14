#ifndef SAVO_SPEECH__AUDIO__PLAYBACK_WORKER_HPP_
#define SAVO_SPEECH__AUDIO__PLAYBACK_WORKER_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/microphone_gate.hpp"
#include "savo_speech/audio/playback_controller.hpp"
#include "savo_speech/audio/playback_sink.hpp"

namespace savo_speech::audio
{

struct PlaybackRequest
{
  std::uint64_t request_id{0U};
  AudioBuffer audio{};
};

enum class PlaybackEnqueueResult : std::uint8_t
{
  Accepted = 0U,
  RejectedNotRunning = 1U,
  RejectedInvalidRequest = 2U,
  RejectedFormatMismatch = 3U,
  RejectedQueueFull = 4U,
  RejectedDuplicateId = 5U
};

[[nodiscard]] constexpr std::string_view to_string(
  const PlaybackEnqueueResult result) noexcept
{
  switch (result) {
    case PlaybackEnqueueResult::Accepted:
      return "accepted";

    case PlaybackEnqueueResult::RejectedNotRunning:
      return "rejected_not_running";

    case PlaybackEnqueueResult::RejectedInvalidRequest:
      return "rejected_invalid_request";

    case PlaybackEnqueueResult::RejectedFormatMismatch:
      return "rejected_format_mismatch";

    case PlaybackEnqueueResult::RejectedQueueFull:
      return "rejected_queue_full";

    case PlaybackEnqueueResult::RejectedDuplicateId:
      return "rejected_duplicate_id";
  }

  return "unknown";
}

enum class PlaybackCompletionStatus : std::uint8_t
{
  Completed = 0U,
  Cancelled = 1U,
  Failed = 2U
};

[[nodiscard]] constexpr std::string_view to_string(
  const PlaybackCompletionStatus status) noexcept
{
  switch (status) {
    case PlaybackCompletionStatus::Completed:
      return "completed";

    case PlaybackCompletionStatus::Cancelled:
      return "cancelled";

    case PlaybackCompletionStatus::Failed:
      return "failed";
  }

  return "unknown";
}

struct PlaybackCompletion
{
  std::uint64_t request_id{0U};

  PlaybackCompletionStatus status{
    PlaybackCompletionStatus::Completed};

  std::size_t frames_submitted{0U};
  std::size_t chunks_submitted{0U};

  std::string error{};
};

enum class PlaybackWorkerState : std::uint8_t
{
  Stopped = 0U,
  Starting = 1U,
  Running = 2U,
  Stopping = 3U,
  Faulted = 4U
};

[[nodiscard]] constexpr std::string_view to_string(
  const PlaybackWorkerState state) noexcept
{
  switch (state) {
    case PlaybackWorkerState::Stopped:
      return "stopped";

    case PlaybackWorkerState::Starting:
      return "starting";

    case PlaybackWorkerState::Running:
      return "running";

    case PlaybackWorkerState::Stopping:
      return "stopping";

    case PlaybackWorkerState::Faulted:
      return "faulted";
  }

  return "unknown";
}

struct PlaybackWorkerConfig
{
  std::size_t queue_capacity{8U};
  std::size_t chunk_frames{320U};

  [[nodiscard]] bool is_valid() const noexcept
  {
    return
      queue_capacity > 0U &&
      queue_capacity <= 1024U &&
      chunk_frames > 0U &&
      chunk_frames <= 65536U;
  }
};

struct PlaybackWorkerStatistics
{
  std::uint64_t starts{0U};
  std::uint64_t stops{0U};

  std::uint64_t enqueue_accepted{0U};
  std::uint64_t enqueue_rejected{0U};

  std::uint64_t completed_requests{0U};
  std::uint64_t cancelled_requests{0U};
  std::uint64_t failed_requests{0U};

  std::uint64_t cancel_current_requests{0U};
  std::uint64_t pending_cancellations{0U};

  std::uint64_t faults{0U};
};

class PlaybackWorker final
{
public:
  PlaybackWorker(
    PlaybackSink & sink,
    MicrophoneGate & microphone_gate,
    PlaybackWorkerConfig config = PlaybackWorkerConfig{});

  ~PlaybackWorker();

  PlaybackWorker(const PlaybackWorker &) = delete;
  PlaybackWorker & operator=(const PlaybackWorker &) = delete;

  PlaybackWorker(PlaybackWorker &&) = delete;
  PlaybackWorker & operator=(PlaybackWorker &&) = delete;

  [[nodiscard]] bool start();
  void stop();

  [[nodiscard]] PlaybackEnqueueResult enqueue(
    PlaybackRequest request);

  [[nodiscard]] bool cancel_current() noexcept;

  [[nodiscard]] std::size_t cancel_pending();

  void cancel_all();

  [[nodiscard]] std::optional<PlaybackCompletion>
  try_pop_completion();

  [[nodiscard]] std::optional<PlaybackCompletion>
  wait_completion_for(std::chrono::milliseconds timeout);

  [[nodiscard]] PlaybackWorkerState state() const noexcept;
  [[nodiscard]] bool running() const noexcept;

  [[nodiscard]] std::size_t pending_requests() const noexcept;

  [[nodiscard]] std::optional<std::uint64_t>
  current_request_id() const noexcept;

  [[nodiscard]] PlaybackWorkerStatistics
  statistics() const noexcept;

  [[nodiscard]] std::string last_error() const;

private:
  void run(std::stop_token stop_token);

  void append_completion(PlaybackCompletion completion);

  [[nodiscard]] bool request_id_exists_locked(
    std::uint64_t request_id) const;

  PlaybackSink & sink_;
  MicrophoneGate & microphone_gate_;

  PlaybackWorkerConfig config_;

  PlaybackController controller_;

  mutable std::mutex mutex_;

  std::condition_variable request_condition_;
  std::condition_variable completion_condition_;

  std::deque<PlaybackRequest> requests_;
  std::deque<PlaybackCompletion> completions_;

  std::jthread thread_;

  PlaybackWorkerState state_{
    PlaybackWorkerState::Stopped};

  bool accepting_requests_{false};

  std::optional<std::uint64_t> current_request_id_{};

  PlaybackWorkerStatistics statistics_{};

  std::string last_error_{};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__PLAYBACK_WORKER_HPP_
