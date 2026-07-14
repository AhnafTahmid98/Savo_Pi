#include "savo_speech/audio/wav_writer.hpp"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace savo_speech::audio
{

namespace
{

void append_tag(
  std::vector<std::uint8_t> & output,
  const std::string_view tag)
{
  if (tag.size() != 4U) {
    throw std::invalid_argument{
            "WAV chunk tag must contain exactly four characters"};
  }

  for (const char character : tag) {
    output.push_back(
      static_cast<std::uint8_t>(
        static_cast<unsigned char>(character)));
  }
}

void append_u16_le(
  std::vector<std::uint8_t> & output,
  const std::uint16_t value)
{
  output.push_back(
    static_cast<std::uint8_t>(value & 0x00FFU));

  output.push_back(
    static_cast<std::uint8_t>((value >> 8U) & 0x00FFU));
}

void append_u32_le(
  std::vector<std::uint8_t> & output,
  const std::uint32_t value)
{
  output.push_back(
    static_cast<std::uint8_t>(value & 0x000000FFU));

  output.push_back(
    static_cast<std::uint8_t>((value >> 8U) & 0x000000FFU));

  output.push_back(
    static_cast<std::uint8_t>((value >> 16U) & 0x000000FFU));

  output.push_back(
    static_cast<std::uint8_t>((value >> 24U) & 0x000000FFU));
}

}  // namespace

std::vector<std::uint8_t> WavWriter::encode(
  const AudioBuffer & audio)
{
  if (!audio.is_consistent()) {
    throw std::invalid_argument{
            "audio buffer is empty, invalid, or channel-inconsistent"};
  }

  if (
    audio.format.sample_format !=
    PcmSampleFormat::Signed16LittleEndian)
  {
    throw std::invalid_argument{
            "WAV writer currently supports only S16_LE PCM"};
  }

  constexpr std::uint32_t kPcmFormatCode{1U};
  constexpr std::uint16_t kBitsPerSample{16U};
  constexpr std::uint32_t kHeaderBytesBeforeData{36U};
  constexpr std::size_t kCompleteHeaderBytes{44U};

  const std::size_t data_size =
    audio.interleaved_samples.size() *
    sizeof(std::int16_t);

  if (
    data_size >
    static_cast<std::size_t>(
      std::numeric_limits<std::uint32_t>::max() -
      kHeaderBytesBeforeData))
  {
    throw std::length_error{
            "PCM audio is too large for a standard RIFF WAV file"};
  }

  const std::uint32_t data_size_u32 =
    static_cast<std::uint32_t>(data_size);

  const std::uint32_t riff_size =
    kHeaderBytesBeforeData + data_size_u32;

  const std::uint16_t block_align =
    static_cast<std::uint16_t>(
      audio.format.channels *
      static_cast<std::uint16_t>(sizeof(std::int16_t)));

  const std::uint64_t byte_rate_wide =
    static_cast<std::uint64_t>(audio.format.sample_rate_hz) *
    static_cast<std::uint64_t>(block_align);

  if (
    byte_rate_wide >
    static_cast<std::uint64_t>(
      std::numeric_limits<std::uint32_t>::max()))
  {
    throw std::length_error{
            "WAV byte rate exceeds the RIFF field capacity"};
  }

  const std::uint32_t byte_rate =
    static_cast<std::uint32_t>(byte_rate_wide);

  std::vector<std::uint8_t> output;
  output.reserve(kCompleteHeaderBytes + data_size);

  append_tag(output, "RIFF");
  append_u32_le(output, riff_size);
  append_tag(output, "WAVE");

  append_tag(output, "fmt ");
  append_u32_le(output, 16U);
  append_u16_le(
    output,
    static_cast<std::uint16_t>(kPcmFormatCode));
  append_u16_le(output, audio.format.channels);
  append_u32_le(output, audio.format.sample_rate_hz);
  append_u32_le(output, byte_rate);
  append_u16_le(output, block_align);
  append_u16_le(output, kBitsPerSample);

  append_tag(output, "data");
  append_u32_le(output, data_size_u32);

  for (const std::int16_t sample :
    audio.interleaved_samples)
  {
    const std::uint16_t encoded =
      static_cast<std::uint16_t>(sample);

    append_u16_le(output, encoded);
  }

  return output;
}

void WavWriter::write_file(
  const std::filesystem::path & path,
  const AudioBuffer & audio)
{
  const std::vector<std::uint8_t> bytes = encode(audio);

  std::ofstream stream{
    path,
    std::ios::binary | std::ios::trunc};

  if (!stream.is_open()) {
    throw std::runtime_error{
            "unable to open WAV output file: " +
            path.string()};
  }

  stream.write(
    reinterpret_cast<const char *>(bytes.data()),
    static_cast<std::streamsize>(bytes.size()));

  if (!stream.good()) {
    throw std::runtime_error{
            "failed while writing WAV output file: " +
            path.string()};
  }
}

}  // namespace savo_speech::audio
