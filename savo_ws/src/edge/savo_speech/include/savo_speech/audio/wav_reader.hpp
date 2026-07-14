#ifndef SAVO_SPEECH__AUDIO__WAV_READER_HPP_
#define SAVO_SPEECH__AUDIO__WAV_READER_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>

#include "savo_speech/audio/audio_buffer.hpp"

namespace savo_speech::audio
{

struct WavReadLimits
{
  std::size_t maximum_file_bytes{
    16U * 1024U * 1024U};

  std::size_t maximum_audio_data_bytes{
    12U * 1024U * 1024U};

  std::uint16_t maximum_channels{32U};

  std::uint32_t minimum_sample_rate_hz{8000U};
  std::uint32_t maximum_sample_rate_hz{192000U};

  [[nodiscard]] bool is_valid() const noexcept
  {
    return
      maximum_file_bytes >= 44U &&
      maximum_audio_data_bytes > 0U &&
      maximum_audio_data_bytes <= maximum_file_bytes &&
      maximum_channels > 0U &&
      minimum_sample_rate_hz > 0U &&
      maximum_sample_rate_hz >= minimum_sample_rate_hz;
  }
};

class WavReader final
{
public:
  WavReader() = delete;

  [[nodiscard]] static AudioBuffer decode(
    std::span<const std::uint8_t> bytes,
    const WavReadLimits & limits = WavReadLimits{});

  [[nodiscard]] static AudioBuffer read_file(
    const std::filesystem::path & path,
    const WavReadLimits & limits = WavReadLimits{});
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__WAV_READER_HPP_
