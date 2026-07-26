#include "savo_speech/session/completed_utterance_queue.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace savo_speech::session
{

CompletedUtteranceQueue::CompletedUtteranceQueue(
  const std::size_t capacity)
: capacity_{capacity}
{
  if (capacity_ == 0U || capacity_ > 1024U) {
    throw std::invalid_argument{
            "completed-utterance queue capacity must be 1..1024"};
  }
}

SerializedUtterancePushResult
CompletedUtteranceQueue::push(
  SerializedUtterance utterance)
{
  if (!utterance.is_valid()) {
    const std::scoped_lock lock{mutex_};

    ++rejected_invalid_;

    return SerializedUtterancePushResult::
           RejectedInvalid;
  }

  {
    const std::scoped_lock lock{mutex_};

    if (contains_id_locked(utterance.utterance_id)) {
      ++rejected_duplicate_id_;

      return SerializedUtterancePushResult::
             RejectedDuplicateId;
    }

    if (queue_.size() >= capacity_) {
      ++rejected_full_;

      return SerializedUtterancePushResult::
             RejectedFull;
    }

    queue_.push_back(std::move(utterance));

    ++accepted_;
  }

  condition_.notify_one();

  return SerializedUtterancePushResult::Accepted;
}

std::optional<SerializedUtterance>
CompletedUtteranceQueue::try_pop()
{
  const std::scoped_lock lock{mutex_};

  if (queue_.empty()) {
    return std::nullopt;
  }

  SerializedUtterance utterance =
    std::move(queue_.front());

  queue_.pop_front();

  ++popped_;

  return utterance;
}

std::optional<SerializedUtterance>
CompletedUtteranceQueue::wait_pop_for(
  const std::chrono::milliseconds timeout)
{
  if (timeout < std::chrono::milliseconds{0}) {
    throw std::invalid_argument{
            "serialized-utterance wait timeout must not be negative"};
  }

  std::unique_lock lock{mutex_};

  const bool available = condition_.wait_for(
    lock,
    timeout,
    [this]() {
      return !queue_.empty();
    });

  if (!available || queue_.empty()) {
    return std::nullopt;
  }

  SerializedUtterance utterance =
    std::move(queue_.front());

  queue_.pop_front();

  ++popped_;

  return utterance;
}

void CompletedUtteranceQueue::clear() noexcept
{
  const std::scoped_lock lock{mutex_};

  cleared_ += static_cast<std::uint64_t>(
    queue_.size());

  queue_.clear();
}

std::size_t CompletedUtteranceQueue::size() const noexcept
{
  const std::scoped_lock lock{mutex_};

  return queue_.size();
}

std::size_t CompletedUtteranceQueue::capacity() const noexcept
{
  return capacity_;
}

bool CompletedUtteranceQueue::empty() const noexcept
{
  return size() == 0U;
}

bool CompletedUtteranceQueue::full() const noexcept
{
  const std::scoped_lock lock{mutex_};

  return queue_.size() >= capacity_;
}

CompletedUtteranceQueueStatistics
CompletedUtteranceQueue::statistics() const noexcept
{
  const std::scoped_lock lock{mutex_};

  CompletedUtteranceQueueStatistics result;

  result.size = queue_.size();
  result.capacity = capacity_;

  result.accepted = accepted_;
  result.rejected_invalid = rejected_invalid_;
  result.rejected_duplicate_id =
    rejected_duplicate_id_;
  result.rejected_full = rejected_full_;
  result.popped = popped_;
  result.cleared = cleared_;

  return result;
}

bool CompletedUtteranceQueue::contains_id_locked(
  const std::uint64_t utterance_id) const noexcept
{
  return std::any_of(
    queue_.begin(),
    queue_.end(),
    [utterance_id](
      const SerializedUtterance & queued)
    {
      return queued.utterance_id == utterance_id;
    });
}

}  // namespace savo_speech::session
