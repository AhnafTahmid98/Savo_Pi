#include "savo_speech/audio/audio_level_meter.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace savo_speech::audio
{

AudioLevel AudioLevelMeter::measure(
  const std::span<const std::int16_t> samples,
  const std::uint32_t clipping_threshold)
{
  constexpr std::uint32_t kMaximumMagnitude{32768U};
  constexpr long double kNormalization{32768.0L};

  if (
    clipping_threshold == 0U ||
    clipping_threshold > kMaximumMagnitude)
  {
    throw std::invalid_argument{
            "clipping_threshold must be in the range [1, 32768]"};
  }

  AudioLevel result;
  result.sample_count = samples.size();

  if (samples.empty()) {
    return result;
  }

  long double squared_sum{0.0L};
  std::uint32_t peak_magnitude{0U};

  for (const std::int16_t sample : samples) {
    const std::int32_t wide_sample =
      static_cast<std::int32_t>(sample);

    const std::uint32_t magnitude =
      wide_sample < 0 ?
      static_cast<std::uint32_t>(-wide_sample) :
      static_cast<std::uint32_t>(wide_sample);

    const long double normalized =
      static_cast<long double>(magnitude) /
      kNormalization;

    squared_sum += normalized * normalized;

    if (magnitude > peak_magnitude) {
      peak_magnitude = magnitude;
    }

    if (magnitude >= clipping_threshold) {
      result.clipping = true;
    }
  }

  const long double mean_square =
    squared_sum /
    static_cast<long double>(samples.size());

  result.rms =
    static_cast<double>(std::sqrt(mean_square));

  result.peak =
    static_cast<double>(
    static_cast<long double>(peak_magnitude) /
    kNormalization);

  return result;
}

}  // namespace savo_speech::audio
