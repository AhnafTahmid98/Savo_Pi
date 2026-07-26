#include <cmath>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

#include "savo_speech/vad/adaptive_energy_vad_backend.hpp"

namespace
{

[[nodiscard]] savo_speech::audio::AudioFrame
make_constant_frame(
  const std::uint64_t sequence,
  const std::int16_t sample,
  const std::uint32_t sample_rate_hz = 16000U,
  const std::uint16_t channels = 1U)
{
  savo_speech::audio::AudioFrame frame;

  frame.sequence = sequence;

  frame.format = {
    sample_rate_hz,
    channels,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  const std::size_t sample_count =
    320U *
    static_cast<std::size_t>(channels);

  frame.interleaved_samples.assign(
    sample_count,
    sample);

  return frame;
}

[[nodiscard]]
savo_speech::vad::AdaptiveEnergyVadConfig
make_config()
{
  savo_speech::vad::AdaptiveEnergyVadConfig config;

  config.expected_sample_rate_hz = 16000U;

  config.startup_calibration_frames = 0U;

  config.initial_noise_floor_rms = 0.010;
  config.minimum_noise_floor_rms = 0.0001;
  config.maximum_noise_floor_rms = 0.500;

  config.noise_floor_update_alpha = 0.50;

  config.minimum_speech_rms = 0.015;

  config.speech_onset_snr_db = 6.0;
  config.speech_saturation_snr_db = 20.0;

  config.clipping_threshold = 32760U;

  return config;
}

[[nodiscard]] double normalized_sample(
  const std::int16_t sample)
{
  return
    std::abs(static_cast<double>(sample)) /
    32768.0;
}

}  // namespace

TEST(
  AdaptiveEnergyVadBackend,
  RejectsInvalidConfiguration)
{
  auto config = make_config();

  config.noise_floor_update_alpha = 0.0;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::vad::
      AdaptiveEnergyVadBackend{config}),
    std::invalid_argument);

  config = make_config();

  config.speech_saturation_snr_db =
    config.speech_onset_snr_db;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::vad::
      AdaptiveEnergyVadBackend{config}),
    std::invalid_argument);

  config = make_config();
  config.clipping_threshold = 32769U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::vad::
      AdaptiveEnergyVadBackend{config}),
    std::invalid_argument);
}

TEST(
  AdaptiveEnergyVadBackend,
  RejectsInvalidFrameFormat)
{
  savo_speech::vad::AdaptiveEnergyVadBackend backend{
    make_config()};

  EXPECT_THROW(
    static_cast<void>(
      backend.analyze(
        make_constant_frame(
          1U,
          100,
          16000U,
          2U))),
    std::invalid_argument);

  EXPECT_THROW(
    static_cast<void>(
      backend.analyze(
        make_constant_frame(
          2U,
          100,
          48000U,
          1U))),
    std::invalid_argument);

  const auto snapshot = backend.snapshot();

  EXPECT_EQ(
    snapshot.statistics.invalid_frames,
    2U);
}

TEST(
  AdaptiveEnergyVadBackend,
  CalibrationSuppressesOutputAndLearnsNoiseFloor)
{
  auto config = make_config();

  config.startup_calibration_frames = 2U;
  config.initial_noise_floor_rms = 0.100;

  savo_speech::vad::AdaptiveEnergyVadBackend backend{
    config};

  const auto first =
    backend.analyze(
    make_constant_frame(1U, 328));

  const auto second =
    backend.analyze(
    make_constant_frame(2U, 655));

  EXPECT_DOUBLE_EQ(first.speech_score, 0.0);
  EXPECT_DOUBLE_EQ(second.speech_score, 0.0);

  const auto snapshot = backend.snapshot();

  const double expected_average =
    (
      normalized_sample(328) +
      normalized_sample(655)
    ) /
    2.0;

  EXPECT_TRUE(snapshot.calibrated);

  EXPECT_EQ(
    snapshot.calibration_frames_completed,
    2U);

  EXPECT_NEAR(
    snapshot.noise_floor_rms,
    expected_average,
    1.0e-9);

  EXPECT_EQ(
    snapshot.statistics.calibration_frames,
    2U);
}

TEST(
  AdaptiveEnergyVadBackend,
  QuietFrameUpdatesNoiseFloor)
{
  auto config = make_config();

  config.initial_noise_floor_rms = 0.020;
  config.minimum_speech_rms = 0.050;
  config.noise_floor_update_alpha = 0.50;

  savo_speech::vad::AdaptiveEnergyVadBackend backend{
    config};

  const auto result =
    backend.analyze(
    make_constant_frame(1U, 328));

  EXPECT_DOUBLE_EQ(result.speech_score, 0.0);

  const double quiet_rms =
    normalized_sample(328);

  const double expected_floor =
    0.50 * 0.020 +
    0.50 * quiet_rms;

  const auto snapshot = backend.snapshot();

  EXPECT_NEAR(
    snapshot.noise_floor_rms,
    expected_floor,
    1.0e-9);

  EXPECT_EQ(
    snapshot.statistics.noise_floor_updates,
    1U);

  EXPECT_EQ(
    snapshot.statistics.noise_floor_freezes,
    0U);
}

TEST(
  AdaptiveEnergyVadBackend,
  SpeechFrameProducesHighScoreAndFreezesNoiseFloor)
{
  auto config = make_config();

  config.initial_noise_floor_rms = 0.010;
  config.minimum_speech_rms = 0.020;

  savo_speech::vad::AdaptiveEnergyVadBackend backend{
    config};

  const auto result =
    backend.analyze(
    make_constant_frame(1U, 3277));

  EXPECT_GT(result.speech_score, 0.95);

  const auto snapshot = backend.snapshot();

  EXPECT_NEAR(
    snapshot.noise_floor_rms,
    0.010,
    1.0e-12);

  EXPECT_EQ(
    snapshot.statistics.noise_floor_freezes,
    1U);

  EXPECT_EQ(
    snapshot.statistics.noise_floor_updates,
    0U);
}

TEST(
  AdaptiveEnergyVadBackend,
  ProducesDeterministicIntermediateScore)
{
  auto config = make_config();

  config.initial_noise_floor_rms = 0.010;
  config.minimum_speech_rms = 0.005;

  savo_speech::vad::AdaptiveEnergyVadBackend backend{
    config};

  const auto result =
    backend.analyze(
    make_constant_frame(1U, 1465));

  EXPECT_GT(result.speech_score, 0.45);
  EXPECT_LT(result.speech_score, 0.55);

  const auto snapshot = backend.snapshot();

  EXPECT_GT(snapshot.last_snr_db, 12.5);
  EXPECT_LT(snapshot.last_snr_db, 13.5);
}

TEST(
  AdaptiveEnergyVadBackend,
  TracksClippingFrames)
{
  savo_speech::vad::AdaptiveEnergyVadBackend backend{
    make_config()};

  static_cast<void>(
    backend.analyze(
      make_constant_frame(
        7U,
        static_cast<std::int16_t>(-32768))));

  const auto snapshot = backend.snapshot();

  EXPECT_TRUE(snapshot.last_frame_clipping);

  EXPECT_DOUBLE_EQ(snapshot.last_peak, 1.0);

  EXPECT_EQ(
    snapshot.statistics.clipping_frames,
    1U);

  EXPECT_EQ(
    snapshot.statistics.last_frame_sequence,
    7U);
}

TEST(
  AdaptiveEnergyVadBackend,
  ResetRestoresInitialState)
{
  auto config = make_config();

  config.initial_noise_floor_rms = 0.020;
  config.minimum_speech_rms = 0.050;

  savo_speech::vad::AdaptiveEnergyVadBackend backend{
    config};

  static_cast<void>(
    backend.analyze(
      make_constant_frame(1U, 328)));

  ASSERT_EQ(
    backend.snapshot().statistics.frames_analyzed,
    1U);

  backend.reset();

  const auto snapshot = backend.snapshot();

  EXPECT_TRUE(snapshot.calibrated);

  EXPECT_EQ(
    snapshot.calibration_frames_completed,
    0U);

  EXPECT_DOUBLE_EQ(
    snapshot.noise_floor_rms,
    config.initial_noise_floor_rms);

  EXPECT_DOUBLE_EQ(snapshot.last_rms, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.last_peak, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.last_snr_db, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.last_speech_score, 0.0);

  EXPECT_FALSE(snapshot.last_frame_clipping);

  EXPECT_EQ(
    snapshot.statistics.frames_analyzed,
    0U);

  EXPECT_TRUE(snapshot.last_error.empty());
}
