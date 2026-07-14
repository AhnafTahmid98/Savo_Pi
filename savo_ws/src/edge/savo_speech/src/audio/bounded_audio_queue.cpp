#include "savo_speech/audio/bounded_audio_queue.hpp"

#include <chrono>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <utility>

namespace savo_speech::audio
{

BoundedAudioQueue::BoundedAudioQueue(
  const std::size_t capacity,
  const QueueOverflowPolicy overflow_policy)
: capacity_{capacity},
  overflow_policy_{overflow_policy}
{
  if (capacity == 0U) {
    throw std::invalid_argument{
            "bounded audio-queue capacity must be greater than zero"};
  }
}

QueuePushResult BoundedAudioQueue::push(AudioFrame frame)
{
  std::unique_lock lock{mutex_};

  if (closed_) {
    ++rejected_closed_frames_;
    return QueuePushResult::RejectedClosed;
  }

  QueuePushResult result = QueuePushResult::Accepted;

  if (queue_.size() >= capacity_) {
    if (overflow_policy_ == QueueOverflowPolicy::RejectNewest) {
      ++rejected_full_frames_;
      return QueuePushResult::RejectedFull;
    }

    queue_.pop_front();
    ++dropped_oldest_frames_;

    result =
      QueuePushResult::AcceptedAfterDroppingOldest;
  }

  queue_.push_back(std::move(frame));
  ++accepted_frames_;

  lock.unlock();
  condition_.notify_one();

  return result;
}

std::optional<AudioFrame> BoundedAudioQueue::try_pop()
{
  const std::scoped_lock lock{mutex_};

  if (queue_.empty()) {
    return std::nullopt;
  }

  AudioFrame frame = std::move(queue_.front());
  queue_.pop_front();

  return frame;
}

std::optional<AudioFrame> BoundedAudioQueue::wait_pop_for(
  const std::chrono::milliseconds timeout)
{
  std::unique_lock lock{mutex_};

  const bool ready = condition_.wait_for(
    lock,
    timeout,
    [this]() {
      return closed_ || !queue_.empty();
    });

  if (!ready || queue_.empty()) {
    return std::nullopt;
  }

  AudioFrame frame = std::move(queue_.front());
  queue_.pop_front();

  return frame;
}

void BoundedAudioQueue::clear() noexcept
{
  const std::scoped_lock lock{mutex_};
  queue_.clear();
}

void BoundedAudioQueue::close() noexcept
{
  {
    const std::scoped_lock lock{mutex_};
    closed_ = true;
  }

  condition_.notify_all();
}

std::size_t BoundedAudioQueue::size() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return queue_.size();
}

std::size_t BoundedAudioQueue::capacity() const noexcept
{
  return capacity_;
}

bool BoundedAudioQueue::empty() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return queue_.empty();
}

bool BoundedAudioQueue::full() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return queue_.size() >= capacity_;
}

bool BoundedAudioQueue::closed() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return closed_;
}

BoundedAudioQueueStatistics
BoundedAudioQueue::statistics() const noexcept
{
  const std::scoped_lock lock{mutex_};

  return BoundedAudioQueueStatistics{
    queue_.size(),
    capacity_,
    closed_,
    accepted_frames_,
    dropped_oldest_frames_,
    rejected_full_frames_,
    rejected_closed_frames_};
}

}  // namespace savo_speech::audio
