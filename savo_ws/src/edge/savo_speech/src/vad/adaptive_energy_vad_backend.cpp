// Copyright 2026 Ahnaf Tahmid
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "savo_speech/vad/adaptive_energy_vad_backend.hpp"

#include "savo_speech/audio/audio_level_meter.hpp"

namespace savo_speech::vad
{

namespace
{

constexpr std::uint32_t kMinimumSampleRateHz{8000U};
constexpr std::uint32_t kMaximumSampleRateHz{192000U};

constexpr std::uint32_t kMaximumPcmMagnitude{32768U};

[[nodiscard]] bool finite_unit_interval(
  const double value) noexcept
{
  return
    std::isfinite(value) &&
    value >= 0.0 &&
    value <= 1.0;
}

[[nodiscard]] double clamp_score(
  const double value) noexcept
{
  return std::clamp(value, 0.0, 1.0);
}

}  // namespace

bool AdaptiveEnergyVadConfig::is_valid() const noexcept
{
  return
    expected_sample_rate_hz >= kMinimumSampleRateHz &&
    expected_sample_rate_hz <= kMaximumSampleRateHz &&
    startup_calibration_frames <= 100000U &&
    finite_unit_interval(initial_noise_floor_rms) &&
    minimum_noise_floor_rms > 0.0 &&
    finite_unit_interval(minimum_noise_floor_rms) &&
    finite_unit_interval(maximum_noise_floor_rms) &&
    minimum_noise_floor_rms <= initial_noise_floor_rms &&
    initial_noise_floor_rms <= maximum_noise_floor_rms &&
    maximum_noise_floor_rms > minimum_noise_floor_rms &&
    std::isfinite(noise_floor_update_alpha) &&
    noise_floor_update_alpha > 0.0 &&
    noise_floor_update_alpha <= 1.0 &&
    finite_unit_interval(minimum_speech_rms) &&
    std::isfinite(speech_onset_snr_db) &&
    std::isfinite(speech_saturation_snr_db) &&
    speech_onset_snr_db >= 0.0 &&
    speech_saturation_snr_db > speech_onset_snr_db &&
    speech_saturation_snr_db <= 100.0 &&
    clipping_threshold >= 1U &&
    clipping_threshold <= kMaximumPcmMagnitude;
}

AdaptiveEnergyVadBackend::AdaptiveEnergyVadBackend(
  AdaptiveEnergyVadConfig config)
: config_{std::move(config)},
  calibrated_{config_.startup_calibration_frames == 0U},
  noise_floor_rms_{config_.initial_noise_floor_rms}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid adaptive-energy VAD configuration"};
  }
}

VadBackendResult AdaptiveEnergyVadBackend::analyze(
  const audio::AudioFrame & frame)
{
  if (!frame.is_consistent()) {
    const std::string error =
      "adaptive-energy VAD received an invalid audio frame";

    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.invalid_frames;
      last_error_ = error;
    }

    throw std::invalid_argument{error};
  }

  if (frame.format.channels != 1U) {
    const std::string error =
      "adaptive-energy VAD requires mono audio";

    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.invalid_frames;
      last_error_ = error;
    }

    throw std::invalid_argument{error};
  }

  if (
    frame.format.sample_rate_hz !=
    config_.expected_sample_rate_hz)
  {
    const std::string error =
      "adaptive-energy VAD received an unexpected "
      "sample rate";

    {
      const std::scoped_lock lock{mutex_};

      ++statistics_.invalid_frames;
      last_error_ = error;
    }

    throw std::invalid_argument{error};
  }

  const auto level =
    audio::AudioLevelMeter::measure(
    frame.interleaved_samples,
    config_.clipping_threshold);

  const std::scoped_lock lock{mutex_};

  ++statistics_.frames_analyzed;

  statistics_.samples_analyzed +=
    static_cast<std::uint64_t>(
    frame.interleaved_samples.size());

  statistics_.last_frame_sequence =
    frame.sequence;

  if (level.clipping) {
    ++statistics_.clipping_frames;
  }

  last_rms_ = level.rms;
  last_peak_ = level.peak;
  last_frame_clipping_ = level.clipping;

  last_error_.clear();

  if (!calibrated_) {
    calibration_rms_sum_ +=
      static_cast<long double>(level.rms);

    ++calibration_frames_completed_;
    ++statistics_.calibration_frames;

    const long double average =
      calibration_rms_sum_ /
      static_cast<long double>(
      calibration_frames_completed_);

    noise_floor_rms_ = std::clamp(
      static_cast<double>(average),
      config_.minimum_noise_floor_rms,
      config_.maximum_noise_floor_rms);

    if (
      calibration_frames_completed_ >=
      config_.startup_calibration_frames)
    {
      calibrated_ = true;
    }

    last_snr_db_ = 0.0;
    last_speech_score_ = 0.0;

    return VadBackendResult{};
  }

  const double snr_db =
    calculate_snr_db_locked(level.rms);

  double speech_score{0.0};

  if (
    level.rms >= config_.minimum_speech_rms &&
    snr_db > config_.speech_onset_snr_db)
  {
    const double score_range =
      config_.speech_saturation_snr_db -
      config_.speech_onset_snr_db;

    speech_score = clamp_score(
      (
        snr_db -
        config_.speech_onset_snr_db
      ) /
      score_range);
  }

  if (speech_score > 0.0) {
    ++statistics_.noise_floor_freezes;
  } else {
    update_noise_floor_locked(level.rms);
    ++statistics_.noise_floor_updates;
  }

  last_snr_db_ = snr_db;
  last_speech_score_ = speech_score;

  VadBackendResult result;
  result.speech_score = speech_score;

  return result;
}

void AdaptiveEnergyVadBackend::reset() noexcept
{
  const std::scoped_lock lock{mutex_};

  statistics_ =
    AdaptiveEnergyVadStatistics{};

  calibrated_ =
    config_.startup_calibration_frames == 0U;

  calibration_frames_completed_ = 0U;
  calibration_rms_sum_ = 0.0L;

  noise_floor_rms_ =
    config_.initial_noise_floor_rms;

  last_rms_ = 0.0;
  last_peak_ = 0.0;
  last_snr_db_ = 0.0;
  last_speech_score_ = 0.0;

  last_frame_clipping_ = false;

  last_error_.clear();
}

AdaptiveEnergyVadSnapshot
AdaptiveEnergyVadBackend::snapshot() const noexcept
{
  const std::scoped_lock lock{mutex_};

  AdaptiveEnergyVadSnapshot snapshot;

  snapshot.calibrated = calibrated_;

  snapshot.calibration_frames_completed =
    calibration_frames_completed_;

  snapshot.noise_floor_rms =
    noise_floor_rms_;

  snapshot.last_rms = last_rms_;
  snapshot.last_peak = last_peak_;
  snapshot.last_snr_db = last_snr_db_;
  snapshot.last_speech_score =
    last_speech_score_;

  snapshot.last_frame_clipping =
    last_frame_clipping_;

  snapshot.statistics = statistics_;
  snapshot.last_error = last_error_;

  return snapshot;
}

double AdaptiveEnergyVadBackend::calculate_snr_db_locked(
  const double rms) const noexcept
{
  const double effective_noise_floor =
    std::max(
    noise_floor_rms_,
    config_.minimum_noise_floor_rms);

  const double effective_rms =
    std::max(
    rms,
    config_.minimum_noise_floor_rms);

  return
    20.0 *
    std::log10(
    effective_rms /
    effective_noise_floor);
}

void AdaptiveEnergyVadBackend::update_noise_floor_locked(
  const double rms) noexcept
{
  const double bounded_rms =
    std::clamp(
    rms,
    config_.minimum_noise_floor_rms,
    config_.maximum_noise_floor_rms);

  noise_floor_rms_ =
    (
    1.0 -
    config_.noise_floor_update_alpha
    ) *
    noise_floor_rms_ +
    config_.noise_floor_update_alpha *
    bounded_rms;

  noise_floor_rms_ =
    std::clamp(
    noise_floor_rms_,
    config_.minimum_noise_floor_rms,
    config_.maximum_noise_floor_rms);
}

}  // namespace savo_speech::vad
