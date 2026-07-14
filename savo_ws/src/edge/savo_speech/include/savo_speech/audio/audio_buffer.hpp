#ifndef SAVO_SPEECH__AUDIO__AUDIO_BUFFER_HPP_
#define SAVO_SPEECH__AUDIO__AUDIO_BUFFER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "savo_speech/audio/audio_format.hpp"

namespace savo_speech::audio
{

struct AudioBuffer
{
  AudioFormat format{};

  // Interleaved signed 16-bit PCM samples.
  std::vector<std::int16_t> interleaved_samples{};

  [[nodiscard]] bool empty() const noexcept
  {
    return interleaved_samples.empty();
  }

  [[nodiscard]] std::size_t frame_count() const noexcept
  {
    if (format.channels == 0U) {
      return 0U;
    }

    return
      interleaved_samples.size() /
      static_cast<std::size_t>(format.channels);
  }

  [[nodiscard]] bool is_consistent() const noexcept
  {
    if (!format.is_valid()) {
      return false;
    }

    if (interleaved_samples.empty()) {
      return false;
    }

    const std::size_t channel_count =
      static_cast<std::size_t>(format.channels);

    return
      interleaved_samples.size() % channel_count == 0U;
  }

  [[nodiscard]] std::chrono::nanoseconds duration() const noexcept
  {
    if (!is_consistent() || format.sample_rate_hz == 0U) {
      return std::chrono::nanoseconds{0};
    }

    constexpr std::uint64_t kNanosecondsPerSecond{
      1000000000ULL};

    const std::uint64_t frames =
      static_cast<std::uint64_t>(frame_count());

    const std::uint64_t duration_nanoseconds =
      frames * kNanosecondsPerSecond /
      static_cast<std::uint64_t>(format.sample_rate_hz);

    return std::chrono::nanoseconds{
      static_cast<std::chrono::nanoseconds::rep>(
        duration_nanoseconds)};
  }
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__AUDIO_BUFFER_HPP_
