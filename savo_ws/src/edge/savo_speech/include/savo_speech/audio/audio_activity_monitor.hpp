#ifndef SAVO_SPEECH__AUDIO__AUDIO_ACTIVITY_MONITOR_HPP_
#define SAVO_SPEECH__AUDIO__AUDIO_ACTIVITY_MONITOR_HPP_

#include <cstdint>
#include <mutex>

#include "savo_speech/audio/captured_audio_processor.hpp"

namespace savo_speech::audio
{

struct AudioActivitySnapshot
{
  std::uint64_t frames_processed{0U};
  std::uint64_t samples_processed{0U};
  std::uint64_t clipping_frames{0U};

  double last_rms{0.0};
  double last_peak{0.0};
  double maximum_peak{0.0};
};

class AudioActivityMonitor final :
  public CapturedAudioProcessor
{
public:
  AudioActivityMonitor() = default;

  void process(const AudioFrame & frame) override;

  [[nodiscard]] AudioActivitySnapshot
  snapshot() const noexcept;

  void reset() noexcept;

private:
  mutable std::mutex mutex_;
  AudioActivitySnapshot snapshot_{};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__AUDIO_ACTIVITY_MONITOR_HPP_
