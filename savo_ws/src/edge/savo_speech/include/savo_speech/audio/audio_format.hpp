#ifndef SAVO_SPEECH__AUDIO__AUDIO_FORMAT_HPP_
#define SAVO_SPEECH__AUDIO__AUDIO_FORMAT_HPP_

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace savo_speech::audio
{

enum class PcmSampleFormat : std::uint8_t
{
  Signed16LittleEndian = 0U
};

[[nodiscard]] constexpr std::string_view to_string(
  const PcmSampleFormat format) noexcept
{
  switch (format) {
    case PcmSampleFormat::Signed16LittleEndian:
      return "S16_LE";
  }

  return "unknown";
}

[[nodiscard]] constexpr std::size_t bytes_per_sample(
  const PcmSampleFormat format) noexcept
{
  switch (format) {
    case PcmSampleFormat::Signed16LittleEndian:
      return sizeof(std::int16_t);
  }

  return 0U;
}

struct AudioFormat
{
  std::uint32_t sample_rate_hz{0U};
  std::uint16_t channels{0U};

  PcmSampleFormat sample_format{
    PcmSampleFormat::Signed16LittleEndian};

  [[nodiscard]] constexpr bool is_valid() const noexcept
  {
    return
      sample_rate_hz >= 8000U &&
      sample_rate_hz <= 192000U &&
      channels >= 1U &&
      channels <= 32U &&
      audio::bytes_per_sample(sample_format) > 0U;
  }

  [[nodiscard]] constexpr std::size_t bytes_per_frame() const noexcept
  {
    return
      static_cast<std::size_t>(channels) *
      audio::bytes_per_sample(sample_format);
  }
};

[[nodiscard]] constexpr bool operator==(
  const AudioFormat & left,
  const AudioFormat & right) noexcept
{
  return
    left.sample_rate_hz == right.sample_rate_hz &&
    left.channels == right.channels &&
    left.sample_format == right.sample_format;
}

[[nodiscard]] constexpr bool operator!=(
  const AudioFormat & left,
  const AudioFormat & right) noexcept
{
  return !(left == right);
}

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__AUDIO_FORMAT_HPP_
