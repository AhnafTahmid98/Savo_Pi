#ifndef SAVO_SPEECH__DRIVERS__ALSA_PLAYBACK_STREAM_HPP_
#define SAVO_SPEECH__DRIVERS__ALSA_PLAYBACK_STREAM_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "savo_speech/audio/audio_format.hpp"
#include "savo_speech/audio/playback_sink.hpp"

namespace savo_speech::drivers
{

struct AlsaPlaybackConfig
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

struct AlsaPlaybackStatistics
{
  std::uint64_t playback_calls{0U};
  std::uint64_t alsa_write_calls{0U};
  std::uint64_t frames_written{0U};

  std::uint64_t short_writes{0U};
  std::uint64_t xrun_recoveries{0U};
  std::uint64_t suspend_recoveries{0U};
  std::uint64_t interrupted_writes{0U};
  std::uint64_t recovery_failures{0U};

  std::uint64_t drain_calls{0U};
  std::uint64_t stop_calls{0U};
};

class AlsaPlaybackStream final : public audio::PlaybackSink
{
public:
  explicit AlsaPlaybackStream(AlsaPlaybackConfig config);

  ~AlsaPlaybackStream() override;

  AlsaPlaybackStream(const AlsaPlaybackStream &) = delete;
  AlsaPlaybackStream & operator=(const AlsaPlaybackStream &) = delete;

  AlsaPlaybackStream(AlsaPlaybackStream &&) = delete;
  AlsaPlaybackStream & operator=(AlsaPlaybackStream &&) = delete;

  void open() override;
  void close() noexcept override;

  [[nodiscard]] bool is_open() const noexcept override;

  [[nodiscard]] audio::AudioFormat format() const noexcept override;

  void write(
    std::span<const std::int16_t> interleaved_samples) override;

  void drain() override;
  void stop() noexcept override;

  [[nodiscard]] std::size_t period_frames() const noexcept;
  [[nodiscard]] std::size_t buffer_frames() const noexcept;

  [[nodiscard]] AlsaPlaybackStatistics statistics() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace savo_speech::drivers

#endif  // SAVO_SPEECH__DRIVERS__ALSA_PLAYBACK_STREAM_HPP_
