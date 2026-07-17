#include "savo_speech/audio/audio_activity_monitor.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>

#include "savo_speech/audio/audio_level_meter.hpp"

namespace savo_speech::audio
{

void AudioActivityMonitor::process(
  const AudioFrame & frame)
{
  if (!frame.is_consistent()) {
    throw std::invalid_argument{
            "audio-activity monitor received an invalid frame"};
  }

  if (frame.format.channels != 1U) {
    throw std::invalid_argument{
            "audio-activity monitor requires mono frames"};
  }

  const auto level =
    AudioLevelMeter::measure(
    frame.interleaved_samples);

  const std::scoped_lock lock{mutex_};

  ++snapshot_.frames_processed;

  snapshot_.samples_processed +=
    static_cast<std::uint64_t>(
    frame.interleaved_samples.size());

  snapshot_.last_rms = level.rms;
  snapshot_.last_peak = level.peak;

  snapshot_.maximum_peak =
    std::max(
    snapshot_.maximum_peak,
    level.peak);

  if (level.clipping) {
    ++snapshot_.clipping_frames;
  }
}

AudioActivitySnapshot
AudioActivityMonitor::snapshot() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return snapshot_;
}

void AudioActivityMonitor::reset() noexcept
{
  const std::scoped_lock lock{mutex_};
  snapshot_ = AudioActivitySnapshot{};
}

}  // namespace savo_speech::audio
