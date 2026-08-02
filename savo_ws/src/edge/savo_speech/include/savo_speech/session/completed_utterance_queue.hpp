// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_QUEUE_HPP_
#define SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_QUEUE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string_view>

#include "savo_speech/session/serialized_utterance.hpp"

namespace savo_speech::session
{

enum class SerializedUtterancePushResult : std::uint8_t
{
  Accepted = 0U,
  RejectedInvalid = 1U,
  RejectedDuplicateId = 2U,
  RejectedFull = 3U
};

[[nodiscard]] constexpr std::string_view to_string(
  const SerializedUtterancePushResult result) noexcept
{
  switch (result) {
    case SerializedUtterancePushResult::Accepted:
      return "accepted";

    case SerializedUtterancePushResult::RejectedInvalid:
      return "rejected_invalid";

    case SerializedUtterancePushResult::RejectedDuplicateId:
      return "rejected_duplicate_id";

    case SerializedUtterancePushResult::RejectedFull:
      return "rejected_full";
  }

  return "unknown";
}

struct CompletedUtteranceQueueStatistics
{
  std::size_t size{0U};
  std::size_t capacity{0U};

  std::uint64_t accepted{0U};
  std::uint64_t rejected_invalid{0U};
  std::uint64_t rejected_duplicate_id{0U};
  std::uint64_t rejected_full{0U};
  std::uint64_t popped{0U};
  std::uint64_t cleared{0U};
};

class CompletedUtteranceQueue final
{
public:
  explicit CompletedUtteranceQueue(
    std::size_t capacity);

  CompletedUtteranceQueue(
    const CompletedUtteranceQueue &) = delete;

  CompletedUtteranceQueue & operator=(
    const CompletedUtteranceQueue &) = delete;

  CompletedUtteranceQueue(
    CompletedUtteranceQueue &&) = delete;

  CompletedUtteranceQueue & operator=(
    CompletedUtteranceQueue &&) = delete;

  [[nodiscard]] SerializedUtterancePushResult push(
    SerializedUtterance utterance);

  [[nodiscard]] std::optional<SerializedUtterance>
  try_pop();

  [[nodiscard]] std::optional<SerializedUtterance>
  wait_pop_for(
    std::chrono::milliseconds timeout);

  void clear() noexcept;

  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] std::size_t capacity() const noexcept;

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool full() const noexcept;

  [[nodiscard]] CompletedUtteranceQueueStatistics
  statistics() const noexcept;

private:
  [[nodiscard]] bool contains_id_locked(
    std::uint64_t utterance_id) const noexcept;

  mutable std::mutex mutex_;
  std::condition_variable condition_;

  std::deque<SerializedUtterance> queue_;

  std::size_t capacity_{0U};

  std::uint64_t accepted_{0U};
  std::uint64_t rejected_invalid_{0U};
  std::uint64_t rejected_duplicate_id_{0U};
  std::uint64_t rejected_full_{0U};
  std::uint64_t popped_{0U};
  std::uint64_t cleared_{0U};
};

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_QUEUE_HPP_
