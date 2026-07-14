#ifndef SAVO_SPEECH__AUDIO__AUDIO_RING_BUFFER_HPP_
#define SAVO_SPEECH__AUDIO__AUDIO_RING_BUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <span>
#include <vector>

namespace savo_speech::audio
{

struct AudioRingBufferStatistics
{
  std::size_t size_samples{0U};
  std::size_t capacity_samples{0U};

  std::uint64_t total_samples_written{0U};
  std::uint64_t overwritten_samples{0U};
};

class AudioRingBuffer final
{
public:
  explicit AudioRingBuffer(std::size_t capacity_samples);

  AudioRingBuffer(const AudioRingBuffer &) = delete;
  AudioRingBuffer & operator=(const AudioRingBuffer &) = delete;

  AudioRingBuffer(AudioRingBuffer &&) = delete;
  AudioRingBuffer & operator=(AudioRingBuffer &&) = delete;

  void append(std::span<const std::int16_t> samples);

  [[nodiscard]] std::vector<std::int16_t> snapshot() const;

  void clear() noexcept;
  void reset_statistics() noexcept;

  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] std::size_t capacity() const noexcept;
  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool full() const noexcept;

  [[nodiscard]] AudioRingBufferStatistics statistics() const noexcept;

private:
  mutable std::mutex mutex_;

  std::vector<std::int16_t> storage_;

  std::size_t write_index_{0U};
  std::size_t size_{0U};

  std::uint64_t total_samples_written_{0U};
  std::uint64_t overwritten_samples_{0U};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__AUDIO_RING_BUFFER_HPP_
