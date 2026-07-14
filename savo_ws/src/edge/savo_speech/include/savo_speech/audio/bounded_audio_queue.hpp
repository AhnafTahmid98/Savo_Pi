#ifndef SAVO_SPEECH__AUDIO__BOUNDED_AUDIO_QUEUE_HPP_
#define SAVO_SPEECH__AUDIO__BOUNDED_AUDIO_QUEUE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

#include "savo_speech/audio/audio_frame.hpp"

namespace savo_speech::audio
{

enum class QueueOverflowPolicy : std::uint8_t
{
  RejectNewest = 0U,
  DropOldest = 1U
};

enum class QueuePushResult : std::uint8_t
{
  Accepted = 0U,
  AcceptedAfterDroppingOldest = 1U,
  RejectedFull = 2U,
  RejectedClosed = 3U
};

struct BoundedAudioQueueStatistics
{
  std::size_t size{0U};
  std::size_t capacity{0U};

  bool closed{false};

  std::uint64_t accepted_frames{0U};
  std::uint64_t dropped_oldest_frames{0U};
  std::uint64_t rejected_full_frames{0U};
  std::uint64_t rejected_closed_frames{0U};
};

class BoundedAudioQueue final
{
public:
  explicit BoundedAudioQueue(
    std::size_t capacity,
    QueueOverflowPolicy overflow_policy =
      QueueOverflowPolicy::DropOldest);

  BoundedAudioQueue(const BoundedAudioQueue &) = delete;
  BoundedAudioQueue & operator=(const BoundedAudioQueue &) = delete;

  BoundedAudioQueue(BoundedAudioQueue &&) = delete;
  BoundedAudioQueue & operator=(BoundedAudioQueue &&) = delete;

  [[nodiscard]] QueuePushResult push(AudioFrame frame);

  [[nodiscard]] std::optional<AudioFrame> try_pop();

  [[nodiscard]] std::optional<AudioFrame> wait_pop_for(
    std::chrono::milliseconds timeout);

  void clear() noexcept;
  void close() noexcept;

  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] std::size_t capacity() const noexcept;
  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool full() const noexcept;
  [[nodiscard]] bool closed() const noexcept;

  [[nodiscard]] BoundedAudioQueueStatistics statistics() const noexcept;

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;

  std::deque<AudioFrame> queue_;

  std::size_t capacity_{0U};

  QueueOverflowPolicy overflow_policy_{
    QueueOverflowPolicy::DropOldest};

  bool closed_{false};

  std::uint64_t accepted_frames_{0U};
  std::uint64_t dropped_oldest_frames_{0U};
  std::uint64_t rejected_full_frames_{0U};
  std::uint64_t rejected_closed_frames_{0U};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__BOUNDED_AUDIO_QUEUE_HPP_
