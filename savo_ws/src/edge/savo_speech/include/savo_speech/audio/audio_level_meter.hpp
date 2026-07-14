#ifndef SAVO_SPEECH__AUDIO__AUDIO_LEVEL_METER_HPP_
#define SAVO_SPEECH__AUDIO__AUDIO_LEVEL_METER_HPP_

#include <cstddef>
#include <cstdint>
#include <span>

namespace savo_speech::audio
{

struct AudioLevel
{
  // Normalized range: 0.0 to 1.0.
  double rms{0.0};
  double peak{0.0};

  bool clipping{false};
  std::size_t sample_count{0U};
};

class AudioLevelMeter final
{
public:
  AudioLevelMeter() = delete;

  [[nodiscard]] static AudioLevel measure(
    std::span<const std::int16_t> samples,
    std::uint32_t clipping_threshold = 32760U);
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__AUDIO_LEVEL_METER_HPP_
