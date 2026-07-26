#ifndef SAVO_SPEECH__VAD__ADAPTIVE_ENERGY_VAD_BACKEND_HPP_
#define SAVO_SPEECH__VAD__ADAPTIVE_ENERGY_VAD_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include "savo_speech/vad/vad_backend.hpp"

namespace savo_speech::vad
{

struct AdaptiveEnergyVadConfig
{
  std::uint32_t expected_sample_rate_hz{16000U};

  std::size_t startup_calibration_frames{25U};

  double initial_noise_floor_rms{0.005};
  double minimum_noise_floor_rms{0.0005};
  double maximum_noise_floor_rms{0.250};

  double noise_floor_update_alpha{0.05};

  double minimum_speech_rms{0.010};

  double speech_onset_snr_db{8.0};
  double speech_saturation_snr_db{20.0};

  std::uint32_t clipping_threshold{32760U};

  [[nodiscard]] bool is_valid() const noexcept;
};

struct AdaptiveEnergyVadStatistics
{
  std::uint64_t frames_analyzed{0U};
  std::uint64_t samples_analyzed{0U};

  std::uint64_t calibration_frames{0U};

  std::uint64_t noise_floor_updates{0U};
  std::uint64_t noise_floor_freezes{0U};

  std::uint64_t clipping_frames{0U};
  std::uint64_t invalid_frames{0U};

  std::uint64_t last_frame_sequence{0U};
};

struct AdaptiveEnergyVadSnapshot
{
  bool calibrated{false};

  std::size_t calibration_frames_completed{0U};

  double noise_floor_rms{0.0};

  double last_rms{0.0};
  double last_peak{0.0};
  double last_snr_db{0.0};
  double last_speech_score{0.0};

  bool last_frame_clipping{false};

  AdaptiveEnergyVadStatistics statistics{};

  std::string last_error{};
};

class AdaptiveEnergyVadBackend final :
  public VadBackend
{
public:
  explicit AdaptiveEnergyVadBackend(
    AdaptiveEnergyVadConfig config =
      AdaptiveEnergyVadConfig{});

  ~AdaptiveEnergyVadBackend() override = default;

  AdaptiveEnergyVadBackend(
    const AdaptiveEnergyVadBackend &) = delete;

  AdaptiveEnergyVadBackend & operator=(
    const AdaptiveEnergyVadBackend &) = delete;

  AdaptiveEnergyVadBackend(
    AdaptiveEnergyVadBackend &&) = delete;

  AdaptiveEnergyVadBackend & operator=(
    AdaptiveEnergyVadBackend &&) = delete;

  [[nodiscard]] VadBackendResult analyze(
    const audio::AudioFrame & frame) override;

  void reset() noexcept override;

  [[nodiscard]] AdaptiveEnergyVadSnapshot
  snapshot() const noexcept;

private:
  [[nodiscard]] double calculate_snr_db_locked(
    double rms) const noexcept;

  void update_noise_floor_locked(
    double rms) noexcept;

  AdaptiveEnergyVadConfig config_;

  mutable std::mutex mutex_;

  AdaptiveEnergyVadStatistics statistics_{};

  bool calibrated_{false};

  std::size_t calibration_frames_completed_{0U};
  long double calibration_rms_sum_{0.0L};

  double noise_floor_rms_{0.0};

  double last_rms_{0.0};
  double last_peak_{0.0};
  double last_snr_db_{0.0};
  double last_speech_score_{0.0};

  bool last_frame_clipping_{false};

  std::string last_error_{};
};

}  // namespace savo_speech::vad

#endif  // SAVO_SPEECH__VAD__ADAPTIVE_ENERGY_VAD_BACKEND_HPP_
