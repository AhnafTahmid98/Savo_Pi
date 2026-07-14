#ifndef SAVO_SPEECH__AUDIO__PLAYBACK_CONTROLLER_HPP_
#define SAVO_SPEECH__AUDIO__PLAYBACK_CONTROLLER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/microphone_gate.hpp"
#include "savo_speech/audio/playback_sink.hpp"

namespace savo_speech::audio
{

enum class PlaybackOutcome : std::uint8_t
{
  Completed = 0U,
  Cancelled = 1U
};

struct PlaybackExecutionResult
{
  PlaybackOutcome outcome{
    PlaybackOutcome::Completed};

  std::size_t frames_submitted{0U};
  std::size_t chunks_submitted{0U};
};

class PlaybackController final
{
public:
  PlaybackController(
    MicrophoneGate & microphone_gate,
    std::size_t chunk_frames);

  PlaybackController(const PlaybackController &) = delete;
  PlaybackController & operator=(const PlaybackController &) = delete;

  PlaybackController(PlaybackController &&) = delete;
  PlaybackController & operator=(PlaybackController &&) = delete;

  [[nodiscard]] PlaybackExecutionResult play(
    const AudioBuffer & audio,
    PlaybackSink & sink);

  void request_cancel() noexcept;
  void clear_cancel() noexcept;

  [[nodiscard]] bool cancel_requested() const noexcept;
  [[nodiscard]] std::size_t chunk_frames() const noexcept;

private:
  MicrophoneGate & microphone_gate_;

  std::size_t chunk_frames_{0U};

  std::atomic_bool cancel_requested_{false};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__PLAYBACK_CONTROLLER_HPP_
