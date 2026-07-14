#ifndef SAVO_SPEECH__AUDIO__WAV_WRITER_HPP_
#define SAVO_SPEECH__AUDIO__WAV_WRITER_HPP_

#include <cstdint>
#include <filesystem>
#include <vector>

#include "savo_speech/audio/audio_buffer.hpp"

namespace savo_speech::audio
{

class WavWriter final
{
public:
  WavWriter() = delete;

  [[nodiscard]] static std::vector<std::uint8_t> encode(
    const AudioBuffer & audio);

  static void write_file(
    const std::filesystem::path & path,
    const AudioBuffer & audio);
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__WAV_WRITER_HPP_
