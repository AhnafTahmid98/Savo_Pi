#include "savo_speech/audio/audio_ring_buffer.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace savo_speech::audio
{

AudioRingBuffer::AudioRingBuffer(
  const std::size_t capacity_samples)
: storage_(capacity_samples)
{
  if (capacity_samples == 0U) {
    throw std::invalid_argument{
            "audio ring-buffer capacity must be greater than zero"};
  }
}

void AudioRingBuffer::append(
  const std::span<const std::int16_t> samples)
{
  if (samples.empty()) {
    return;
  }

  const std::scoped_lock lock{mutex_};

  total_samples_written_ +=
    static_cast<std::uint64_t>(samples.size());

  const std::size_t capacity_samples = storage_.size();

  if (samples.size() >= capacity_samples) {
    const std::size_t discarded_samples =
      size_ + samples.size() - capacity_samples;

    overwritten_samples_ +=
      static_cast<std::uint64_t>(discarded_samples);

    const auto first_retained_sample =
      samples.end() -
      static_cast<std::ptrdiff_t>(capacity_samples);

    std::copy(
      first_retained_sample,
      samples.end(),
      storage_.begin());

    size_ = capacity_samples;
    write_index_ = 0U;
    return;
  }

  const std::size_t available_samples =
    capacity_samples - size_;

  if (samples.size() > available_samples) {
    overwritten_samples_ +=
      static_cast<std::uint64_t>(
      samples.size() - available_samples);
  }

  for (const std::int16_t sample : samples) {
    storage_[write_index_] = sample;

    write_index_ =
      (write_index_ + 1U) % capacity_samples;

    if (size_ < capacity_samples) {
      ++size_;
    }
  }
}

std::vector<std::int16_t> AudioRingBuffer::snapshot() const
{
  const std::scoped_lock lock{mutex_};

  std::vector<std::int16_t> result;
  result.reserve(size_);

  if (size_ == 0U) {
    return result;
  }

  const std::size_t capacity_samples = storage_.size();

  const std::size_t oldest_index =
    size_ == capacity_samples ?
    write_index_ :
    0U;

  for (std::size_t index = 0U; index < size_; ++index) {
    result.push_back(
      storage_[
        (oldest_index + index) % capacity_samples]);
  }

  return result;
}

void AudioRingBuffer::clear() noexcept
{
  const std::scoped_lock lock{mutex_};

  write_index_ = 0U;
  size_ = 0U;
}

void AudioRingBuffer::reset_statistics() noexcept
{
  const std::scoped_lock lock{mutex_};

  total_samples_written_ = 0U;
  overwritten_samples_ = 0U;
}

std::size_t AudioRingBuffer::size() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return size_;
}

std::size_t AudioRingBuffer::capacity() const noexcept
{
  return storage_.size();
}

bool AudioRingBuffer::empty() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return size_ == 0U;
}

bool AudioRingBuffer::full() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return size_ == storage_.size();
}

AudioRingBufferStatistics
AudioRingBuffer::statistics() const noexcept
{
  const std::scoped_lock lock{mutex_};

  return AudioRingBufferStatistics{
    size_,
    storage_.size(),
    total_samples_written_,
    overwritten_samples_};
}

}  // namespace savo_speech::audio
