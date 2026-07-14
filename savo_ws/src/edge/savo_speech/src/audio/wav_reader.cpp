#include "savo_speech/audio/wav_reader.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace savo_speech::audio
{

namespace
{

[[nodiscard]] bool tag_equals(
  const std::span<const std::uint8_t> bytes,
  const std::size_t offset,
  const std::string_view expected)
{
  if (
    expected.size() != 4U ||
    offset > bytes.size() ||
    bytes.size() - offset < 4U)
  {
    return false;
  }

  for (std::size_t index = 0U; index < 4U; ++index) {
    if (
      bytes[offset + index] !=
      static_cast<std::uint8_t>(
        static_cast<unsigned char>(expected[index])))
    {
      return false;
    }
  }

  return true;
}

[[nodiscard]] std::uint16_t read_u16_le(
  const std::span<const std::uint8_t> bytes,
  const std::size_t offset)
{
  if (offset > bytes.size() || bytes.size() - offset < 2U) {
    throw std::invalid_argument{
            "truncated 16-bit WAV field"};
  }

  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(bytes[offset]) |
    static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(bytes[offset + 1U])
      << 8U));
}

[[nodiscard]] std::uint32_t read_u32_le(
  const std::span<const std::uint8_t> bytes,
  const std::size_t offset)
{
  if (offset > bytes.size() || bytes.size() - offset < 4U) {
    throw std::invalid_argument{
            "truncated 32-bit WAV field"};
  }

  return
    static_cast<std::uint32_t>(bytes[offset]) |
    (static_cast<std::uint32_t>(bytes[offset + 1U]) << 8U) |
    (static_cast<std::uint32_t>(bytes[offset + 2U]) << 16U) |
    (static_cast<std::uint32_t>(bytes[offset + 3U]) << 24U);
}

struct ParsedFormat
{
  bool present{false};

  std::uint16_t format_code{0U};
  std::uint16_t channels{0U};
  std::uint32_t sample_rate_hz{0U};
  std::uint32_t byte_rate{0U};
  std::uint16_t block_align{0U};
  std::uint16_t bits_per_sample{0U};
};

void validate_format(
  const ParsedFormat & format,
  const WavReadLimits & limits)
{
  constexpr std::uint16_t kPcmFormatCode{1U};
  constexpr std::uint16_t kBitsPerSample{16U};

  if (!format.present) {
    throw std::invalid_argument{
            "WAV file does not contain a fmt chunk"};
  }

  if (format.format_code != kPcmFormatCode) {
    throw std::invalid_argument{
            "WAV file is not uncompressed PCM"};
  }

  if (format.bits_per_sample != kBitsPerSample) {
    throw std::invalid_argument{
            "WAV reader supports only 16-bit PCM"};
  }

  if (
    format.channels == 0U ||
    format.channels > limits.maximum_channels)
  {
    throw std::invalid_argument{
            "WAV channel count is outside the allowed range"};
  }

  if (
    format.sample_rate_hz < limits.minimum_sample_rate_hz ||
    format.sample_rate_hz > limits.maximum_sample_rate_hz)
  {
    throw std::invalid_argument{
            "WAV sample rate is outside the allowed range"};
  }

  const std::uint16_t expected_block_align =
    static_cast<std::uint16_t>(
      format.channels * sizeof(std::int16_t));

  if (format.block_align != expected_block_align) {
    throw std::invalid_argument{
            "WAV block alignment does not match its channel count"};
  }

  const std::uint64_t expected_byte_rate =
    static_cast<std::uint64_t>(format.sample_rate_hz) *
    static_cast<std::uint64_t>(expected_block_align);

  if (
    expected_byte_rate !=
    static_cast<std::uint64_t>(format.byte_rate))
  {
    throw std::invalid_argument{
            "WAV byte rate does not match its format"};
  }
}

}  // namespace

AudioBuffer WavReader::decode(
  const std::span<const std::uint8_t> bytes,
  const WavReadLimits & limits)
{
  if (!limits.is_valid()) {
    throw std::invalid_argument{
            "invalid WAV read limits"};
  }

  if (bytes.size() > limits.maximum_file_bytes) {
    throw std::length_error{
            "WAV input exceeds the configured file-size limit"};
  }

  if (bytes.size() < 12U) {
    throw std::invalid_argument{
            "WAV input is shorter than the RIFF header"};
  }

  if (!tag_equals(bytes, 0U, "RIFF")) {
    throw std::invalid_argument{
            "WAV input is missing the RIFF signature"};
  }

  if (!tag_equals(bytes, 8U, "WAVE")) {
    throw std::invalid_argument{
            "RIFF input is missing the WAVE signature"};
  }

  const std::uint32_t declared_riff_size =
    read_u32_le(bytes, 4U);

  if (
    declared_riff_size >
    std::numeric_limits<std::size_t>::max() - 8U)
  {
    throw std::length_error{
            "WAV RIFF size overflows the platform size type"};
  }

  const std::size_t riff_end =
    8U + static_cast<std::size_t>(declared_riff_size);

  if (riff_end > bytes.size()) {
    throw std::invalid_argument{
            "WAV RIFF payload is truncated"};
  }

  if (riff_end < 12U) {
    throw std::invalid_argument{
            "WAV RIFF payload is invalid"};
  }

  ParsedFormat parsed_format;

  bool data_present{false};
  std::size_t data_offset{0U};
  std::size_t data_size{0U};

  std::size_t offset{12U};

  while (offset + 8U <= riff_end) {
    const std::uint32_t chunk_size_u32 =
      read_u32_le(bytes, offset + 4U);

    const std::size_t chunk_size =
      static_cast<std::size_t>(chunk_size_u32);

    const std::size_t chunk_data_offset = offset + 8U;

    if (
      chunk_size >
      riff_end - chunk_data_offset)
    {
      throw std::invalid_argument{
              "WAV chunk extends beyond the RIFF payload"};
    }

    const std::size_t chunk_end =
      chunk_data_offset + chunk_size;

    if (tag_equals(bytes, offset, "fmt ")) {
      if (parsed_format.present) {
        throw std::invalid_argument{
                "WAV file contains multiple fmt chunks"};
      }

      if (chunk_size < 16U) {
        throw std::invalid_argument{
                "WAV fmt chunk is shorter than PCM format data"};
      }

      parsed_format.present = true;
      parsed_format.format_code =
        read_u16_le(bytes, chunk_data_offset);
      parsed_format.channels =
        read_u16_le(bytes, chunk_data_offset + 2U);
      parsed_format.sample_rate_hz =
        read_u32_le(bytes, chunk_data_offset + 4U);
      parsed_format.byte_rate =
        read_u32_le(bytes, chunk_data_offset + 8U);
      parsed_format.block_align =
        read_u16_le(bytes, chunk_data_offset + 12U);
      parsed_format.bits_per_sample =
        read_u16_le(bytes, chunk_data_offset + 14U);
    } else if (tag_equals(bytes, offset, "data")) {
      if (data_present) {
        throw std::invalid_argument{
                "WAV file contains multiple data chunks"};
      }

      if (
        chunk_size >
        limits.maximum_audio_data_bytes)
      {
        throw std::length_error{
                "WAV audio data exceeds the configured limit"};
      }

      data_present = true;
      data_offset = chunk_data_offset;
      data_size = chunk_size;
    }

    const std::size_t padding =
      chunk_size % 2U;

    if (chunk_end > riff_end - padding) {
      throw std::invalid_argument{
              "WAV chunk padding extends beyond RIFF payload"};
    }

    offset = chunk_end + padding;
  }

  validate_format(parsed_format, limits);

  if (!data_present) {
    throw std::invalid_argument{
            "WAV file does not contain a data chunk"};
  }

  if (data_size == 0U) {
    throw std::invalid_argument{
            "WAV data chunk is empty"};
  }

  if (
    data_size %
    static_cast<std::size_t>(parsed_format.block_align) != 0U)
  {
    throw std::invalid_argument{
            "WAV data size is not frame-aligned"};
  }

  if (data_size % sizeof(std::int16_t) != 0U) {
    throw std::invalid_argument{
            "WAV data size is not 16-bit sample-aligned"};
  }

  AudioBuffer result;

  result.format = AudioFormat{
    parsed_format.sample_rate_hz,
    parsed_format.channels,
    PcmSampleFormat::Signed16LittleEndian};

  const std::size_t sample_count =
    data_size / sizeof(std::int16_t);

  result.interleaved_samples.reserve(sample_count);

  for (std::size_t index = 0U;
    index < sample_count;
    ++index)
  {
    const std::uint16_t encoded =
      read_u16_le(
      bytes,
      data_offset + index * sizeof(std::int16_t));

    result.interleaved_samples.push_back(
      static_cast<std::int16_t>(encoded));
  }

  if (!result.is_consistent()) {
    throw std::invalid_argument{
            "decoded WAV produced an inconsistent audio buffer"};
  }

  return result;
}

AudioBuffer WavReader::read_file(
  const std::filesystem::path & path,
  const WavReadLimits & limits)
{
  std::ifstream stream{
    path,
    std::ios::binary | std::ios::ate};

  if (!stream.is_open()) {
    throw std::runtime_error{
            "unable to open WAV input file: " +
            path.string()};
  }

  const std::streampos end_position = stream.tellg();

  if (end_position < 0) {
    throw std::runtime_error{
            "unable to determine WAV input-file size: " +
            path.string()};
  }

  const auto file_size =
    static_cast<std::uintmax_t>(end_position);

  if (
    file_size >
    static_cast<std::uintmax_t>(limits.maximum_file_bytes))
  {
    throw std::length_error{
            "WAV input file exceeds the configured size limit"};
  }

  std::vector<std::uint8_t> bytes(
    static_cast<std::size_t>(file_size));

  stream.seekg(0, std::ios::beg);

  if (!bytes.empty()) {
    stream.read(
      reinterpret_cast<char *>(bytes.data()),
      static_cast<std::streamsize>(bytes.size()));
  }

  if (!stream.good() && !stream.eof()) {
    throw std::runtime_error{
            "failed while reading WAV input file: " +
            path.string()};
  }

  return decode(bytes, limits);
}

}  // namespace savo_speech::audio
