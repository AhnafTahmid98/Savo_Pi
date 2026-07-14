#ifndef SAVO_SPEECH__AUDIO__PLAYBACK_SINK_HPP_
#define SAVO_SPEECH__AUDIO__PLAYBACK_SINK_HPP_

#include <cstdint>
#include <span>

#include "savo_speech/audio/audio_format.hpp"

namespace savo_speech::audio
{

class PlaybackSink
{
public:
  virtual ~PlaybackSink() = default;

  PlaybackSink(const PlaybackSink &) = delete;
  PlaybackSink & operator=(const PlaybackSink &) = delete;

  PlaybackSink(PlaybackSink &&) = delete;
  PlaybackSink & operator=(PlaybackSink &&) = delete;

  virtual void open() = 0;
  virtual void close() noexcept = 0;

  [[nodiscard]] virtual bool is_open() const noexcept = 0;

  [[nodiscard]] virtual AudioFormat format() const noexcept = 0;

  virtual void write(
    std::span<const std::int16_t> interleaved_samples) = 0;

  virtual void drain() = 0;
  virtual void stop() noexcept = 0;

protected:
  PlaybackSink() = default;
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__PLAYBACK_SINK_HPP_
