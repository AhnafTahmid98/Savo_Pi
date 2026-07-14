#ifndef SAVO_SPEECH__DRIVERS__ALSA_CAPTURE_STREAM_HPP_
#define SAVO_SPEECH__DRIVERS__ALSA_CAPTURE_STREAM_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "savo_speech/audio/audio_format.hpp"
#include "savo_speech/audio/audio_frame.hpp"
#include "savo_speech/audio/capture_source.hpp"

namespace savo_speech::drivers
{

struct AlsaCaptureConfig
{
  std::string device_name{};

  audio::AudioFormat requested_format{
    16000U,
    1U,
    audio::PcmSampleFormat::Signed16LittleEndian};

  std::size_t period_frames{320U};
  std::size_t periods{4U};

  bool require_exact_sample_rate{true};

  std::size_t maximum_recovery_attempts{3U};

  [[nodiscard]] bool is_valid() const noexcept
  {
    return
      !device_name.empty() &&
      requested_format.is_valid() &&
      period_frames > 0U &&
      period_frames <= 65536U &&
      periods >= 2U &&
      periods <= 32U &&
      maximum_recovery_attempts > 0U &&
      maximum_recovery_attempts <= 100U;
  }
};

struct AlsaCaptureStatistics
{
  std::uint64_t capture_calls{0U};
  std::uint64_t alsa_read_calls{0U};
  std::uint64_t frames_captured{0U};

  std::uint64_t short_reads{0U};
  std::uint64_t xrun_recoveries{0U};
  std::uint64_t suspend_recoveries{0U};
  std::uint64_t interrupted_reads{0U};
  std::uint64_t recovery_failures{0U};
};

class AlsaCaptureStream final : public audio::CaptureSource
{
public:
  explicit AlsaCaptureStream(AlsaCaptureConfig config);

  ~AlsaCaptureStream() override;

  AlsaCaptureStream(const AlsaCaptureStream &) = delete;
  AlsaCaptureStream & operator=(const AlsaCaptureStream &) = delete;

  AlsaCaptureStream(AlsaCaptureStream &&) = delete;
  AlsaCaptureStream & operator=(AlsaCaptureStream &&) = delete;

  void open() override;
  void close() noexcept override;

  [[nodiscard]] bool is_open() const noexcept override;

  [[nodiscard]] audio::AudioFormat format() const noexcept override;

  [[nodiscard]] audio::AudioFrame read_frame() override;

  [[nodiscard]] std::size_t period_frames() const noexcept;
  [[nodiscard]] std::size_t buffer_frames() const noexcept;

  [[nodiscard]] AlsaCaptureStatistics statistics() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace savo_speech::drivers

#endif  // SAVO_SPEECH__DRIVERS__ALSA_CAPTURE_STREAM_HPP_
