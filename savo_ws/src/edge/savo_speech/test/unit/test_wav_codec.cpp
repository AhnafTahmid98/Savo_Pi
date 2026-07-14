#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/wav_reader.hpp"
#include "savo_speech/audio/wav_writer.hpp"

namespace
{

savo_speech::audio::AudioBuffer make_mono_audio()
{
  savo_speech::audio::AudioBuffer audio;

  audio.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  audio.interleaved_samples = {
    static_cast<std::int16_t>(0),
    static_cast<std::int16_t>(1),
    static_cast<std::int16_t>(-1),
    static_cast<std::int16_t>(16384),
    static_cast<std::int16_t>(-16384),
    static_cast<std::int16_t>(32767),
    static_cast<std::int16_t>(-32768)};

  return audio;
}

void write_u32_le(
  std::vector<std::uint8_t> & bytes,
  const std::size_t offset,
  const std::uint32_t value)
{
  ASSERT_GE(bytes.size(), offset + 4U);

  bytes[offset] =
    static_cast<std::uint8_t>(value & 0xFFU);

  bytes[offset + 1U] =
    static_cast<std::uint8_t>((value >> 8U) & 0xFFU);

  bytes[offset + 2U] =
    static_cast<std::uint8_t>((value >> 16U) & 0xFFU);

  bytes[offset + 3U] =
    static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

}  // namespace

TEST(WavCodec, EncodedMonoHeaderHasExpectedSize)
{
  const auto audio = make_mono_audio();

  const auto bytes =
    savo_speech::audio::WavWriter::encode(audio);

  EXPECT_EQ(
    bytes.size(),
    44U + audio.interleaved_samples.size() * 2U);

  ASSERT_GE(bytes.size(), 44U);

  EXPECT_EQ(bytes[0], static_cast<std::uint8_t>('R'));
  EXPECT_EQ(bytes[1], static_cast<std::uint8_t>('I'));
  EXPECT_EQ(bytes[2], static_cast<std::uint8_t>('F'));
  EXPECT_EQ(bytes[3], static_cast<std::uint8_t>('F'));

  EXPECT_EQ(bytes[8], static_cast<std::uint8_t>('W'));
  EXPECT_EQ(bytes[9], static_cast<std::uint8_t>('A'));
  EXPECT_EQ(bytes[10], static_cast<std::uint8_t>('V'));
  EXPECT_EQ(bytes[11], static_cast<std::uint8_t>('E'));
}

TEST(WavCodec, MonoRoundTripPreservesFormatAndSamples)
{
  const auto expected = make_mono_audio();

  const auto bytes =
    savo_speech::audio::WavWriter::encode(expected);

  const auto actual =
    savo_speech::audio::WavReader::decode(bytes);

  EXPECT_EQ(actual.format, expected.format);
  EXPECT_EQ(
    actual.interleaved_samples,
    expected.interleaved_samples);
}

TEST(WavCodec, StereoRoundTripPreservesInterleaving)
{
  savo_speech::audio::AudioBuffer expected;

  expected.format = {
    48000U,
    2U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  expected.interleaved_samples = {
    static_cast<std::int16_t>(10),
    static_cast<std::int16_t>(20),
    static_cast<std::int16_t>(11),
    static_cast<std::int16_t>(21)};

  const auto bytes =
    savo_speech::audio::WavWriter::encode(expected);

  const auto actual =
    savo_speech::audio::WavReader::decode(bytes);

  EXPECT_EQ(actual.format, expected.format);
  EXPECT_EQ(
    actual.interleaved_samples,
    expected.interleaved_samples);
}

TEST(WavCodec, RejectsEmptyAudioDuringEncoding)
{
  savo_speech::audio::AudioBuffer audio;

  audio.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::WavWriter::encode(audio)),
    std::invalid_argument);
}

TEST(WavCodec, RejectsTruncatedHeader)
{
  const std::vector<std::uint8_t> bytes{
    static_cast<std::uint8_t>('R'),
    static_cast<std::uint8_t>('I'),
    static_cast<std::uint8_t>('F'),
    static_cast<std::uint8_t>('F')};

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::WavReader::decode(bytes)),
    std::invalid_argument);
}

TEST(WavCodec, RejectsWrongRiffSignature)
{
  const auto audio = make_mono_audio();
  auto bytes =
    savo_speech::audio::WavWriter::encode(audio);

  bytes[0] = static_cast<std::uint8_t>('X');

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::WavReader::decode(bytes)),
    std::invalid_argument);
}

TEST(WavCodec, RejectsWrongWaveSignature)
{
  const auto audio = make_mono_audio();
  auto bytes =
    savo_speech::audio::WavWriter::encode(audio);

  bytes[8] = static_cast<std::uint8_t>('X');

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::WavReader::decode(bytes)),
    std::invalid_argument);
}

TEST(WavCodec, RejectsUnsupportedBitDepth)
{
  const auto audio = make_mono_audio();
  auto bytes =
    savo_speech::audio::WavWriter::encode(audio);

  bytes[34] = 8U;
  bytes[35] = 0U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::WavReader::decode(bytes)),
    std::invalid_argument);
}

TEST(WavCodec, RejectsTruncatedRiffPayload)
{
  const auto audio = make_mono_audio();
  auto bytes =
    savo_speech::audio::WavWriter::encode(audio);

  write_u32_le(
    bytes,
    4U,
    static_cast<std::uint32_t>(bytes.size() + 100U));

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::WavReader::decode(bytes)),
    std::invalid_argument);
}

TEST(WavCodec, RejectsDataThatIsNotFrameAligned)
{
  savo_speech::audio::AudioBuffer audio;

  audio.format = {
    48000U,
    2U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  audio.interleaved_samples = {
    static_cast<std::int16_t>(1),
    static_cast<std::int16_t>(2)};

  auto bytes =
    savo_speech::audio::WavWriter::encode(audio);

  write_u32_le(bytes, 40U, 2U);
  write_u32_le(bytes, 4U, 38U);
  bytes.resize(46U);

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::WavReader::decode(bytes)),
    std::invalid_argument);
}

TEST(WavCodec, EnforcesConfiguredFileSizeLimit)
{
  const auto audio = make_mono_audio();

  const auto bytes =
    savo_speech::audio::WavWriter::encode(audio);

  savo_speech::audio::WavReadLimits limits;
  limits.maximum_file_bytes = 44U;
  limits.maximum_audio_data_bytes = 44U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::WavReader::decode(bytes, limits)),
    std::length_error);
}
