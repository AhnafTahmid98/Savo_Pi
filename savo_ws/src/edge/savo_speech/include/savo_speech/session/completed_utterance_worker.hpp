#ifndef SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_WORKER_HPP_
#define SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_WORKER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>

#include "savo_speech/session/completed_utterance_queue.hpp"
#include "savo_speech/session/completed_utterance_source.hpp"

namespace savo_speech::session
{

enum class CompletedUtteranceWorkerState : std::uint8_t
{
  Stopped = 0U,
  Starting = 1U,
  Running = 2U,
  Stopping = 3U,
  Faulted = 4U
};

[[nodiscard]] constexpr std::string_view to_string(
  const CompletedUtteranceWorkerState state) noexcept
{
  switch (state) {
    case CompletedUtteranceWorkerState::Stopped:
      return "stopped";

    case CompletedUtteranceWorkerState::Starting:
      return "starting";

    case CompletedUtteranceWorkerState::Running:
      return "running";

    case CompletedUtteranceWorkerState::Stopping:
      return "stopping";

    case CompletedUtteranceWorkerState::Faulted:
      return "faulted";
  }

  return "unknown";
}

struct CompletedUtteranceWorkerConfig
{
  std::size_t output_queue_capacity{4U};

  std::chrono::milliseconds source_wait_timeout{
    std::chrono::milliseconds{100}};

  std::size_t maximum_wav_bytes{
    2U * 1024U * 1024U};

  [[nodiscard]] bool is_valid() const noexcept
  {
    return
      output_queue_capacity > 0U &&
      output_queue_capacity <= 1024U &&
      source_wait_timeout >= std::chrono::milliseconds{1} &&
      source_wait_timeout <= std::chrono::milliseconds{5000} &&
      maximum_wav_bytes >= 44U &&
      maximum_wav_bytes <= 64U * 1024U * 1024U;
  }
};

struct CompletedUtteranceWorkerStatistics
{
  std::uint64_t starts{0U};
  std::uint64_t stops{0U};

  std::uint64_t source_wait_timeouts{0U};

  std::uint64_t utterances_received{0U};
  std::uint64_t utterances_serialized{0U};

  std::uint64_t invalid_utterances{0U};
  std::uint64_t duplicate_or_out_of_order_ids{0U};

  std::uint64_t encoding_failures{0U};
  std::uint64_t wav_size_limit_rejections{0U};
  std::uint64_t output_queue_rejections{0U};

  std::uint64_t faults{0U};
};

struct CompletedUtteranceWorkerSnapshot
{
  CompletedUtteranceWorkerState state{
    CompletedUtteranceWorkerState::Stopped};

  CompletedUtteranceWorkerStatistics statistics{};
  CompletedUtteranceQueueStatistics output_queue{};

  std::optional<std::uint64_t> current_utterance_id{};
  std::optional<std::uint64_t> last_seen_utterance_id{};

  std::string last_error{};
};

class CompletedUtteranceWorker final
{
public:
  CompletedUtteranceWorker(
    CompletedUtteranceSource & source,
    CompletedUtteranceWorkerConfig config =
      CompletedUtteranceWorkerConfig{});

  ~CompletedUtteranceWorker();

  CompletedUtteranceWorker(
    const CompletedUtteranceWorker &) = delete;

  CompletedUtteranceWorker & operator=(
    const CompletedUtteranceWorker &) = delete;

  CompletedUtteranceWorker(
    CompletedUtteranceWorker &&) = delete;

  CompletedUtteranceWorker & operator=(
    CompletedUtteranceWorker &&) = delete;

  [[nodiscard]] bool start();
  void stop();

  [[nodiscard]] bool running() const noexcept;

  [[nodiscard]] std::optional<SerializedUtterance>
  try_pop_serialized();

  [[nodiscard]] std::optional<SerializedUtterance>
  wait_serialized_for(
    std::chrono::milliseconds timeout);

  void clear_output() noexcept;

  [[nodiscard]] CompletedUtteranceWorkerSnapshot
  snapshot() const;

private:
  void run(std::stop_token stop_token);

  void process_completed(
    CompletedUtterance utterance);

  void fault(std::string error) noexcept;

  void clear_current() noexcept;

  CompletedUtteranceSource & source_;
  CompletedUtteranceWorkerConfig config_;

  CompletedUtteranceQueue output_queue_;

  mutable std::mutex mutex_;

  std::jthread thread_;

  CompletedUtteranceWorkerState state_{
    CompletedUtteranceWorkerState::Stopped};

  CompletedUtteranceWorkerStatistics statistics_{};

  std::optional<std::uint64_t> current_utterance_id_{};
  std::optional<std::uint64_t> last_seen_utterance_id_{};

  std::string last_error_{};
};

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_WORKER_HPP_
