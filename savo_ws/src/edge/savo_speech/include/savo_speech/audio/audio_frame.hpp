#ifndef SAVO_SPEECH__AUDIO__AUDIO_FRAME_HPP_
#define SAVO_SPEECH__AUDIO__AUDIO_FRAME_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "savo_speech/audio/audio_format.hpp"

namespace savo_speech::audio
{

struct AudioFrame
{
  using Clock = std::chrono::steady_clock;

  std::uint64_t sequence{0U};
  Clock::time_point captured_at{};

  AudioFormat format{};

  // Interleaved signed 16-bit PCM samples.
  //
  // Stereo example:
  // L0, R0, L1, R1, L2, R2
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

    const std::size_t channel_count =
      static_cast<std::size_t>(format.channels);

    return
      !interleaved_samples.empty() &&
      interleaved_samples.size() % channel_count == 0U;
  }
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__AUDIO_FRAME_HPP_
