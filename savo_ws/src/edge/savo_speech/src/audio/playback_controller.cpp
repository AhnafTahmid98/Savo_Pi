#include "savo_speech/audio/playback_controller.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>

namespace savo_speech::audio
{

PlaybackController::PlaybackController(
  MicrophoneGate & microphone_gate,
  const std::size_t chunk_frames)
: microphone_gate_{microphone_gate},
  chunk_frames_{chunk_frames}
{
  if (chunk_frames_ == 0U) {
    throw std::invalid_argument{
            "playback chunk size must be greater than zero"};
  }
}

PlaybackExecutionResult PlaybackController::play(
  const AudioBuffer & audio,
  PlaybackSink & sink)
{
  if (!audio.is_consistent()) {
    throw std::invalid_argument{
            "playback audio buffer is invalid or inconsistent"};
  }

  if (!sink.is_open()) {
    throw std::logic_error{
            "playback sink is not open"};
  }

  if (sink.format() != audio.format) {
    throw std::invalid_argument{
            "playback sink format does not match audio format"};
  }

  const std::size_t channel_count =
    static_cast<std::size_t>(audio.format.channels);

  if (
    chunk_frames_ >
    std::numeric_limits<std::size_t>::max() /
    channel_count)
  {
    throw std::length_error{
            "playback chunk sample count overflows size_t"};
  }

  microphone_gate_.begin_playback();

  PlaybackExecutionResult result;

  const std::span<const std::int16_t> samples{
    audio.interleaved_samples};

  const std::size_t total_frames = audio.frame_count();

  try {
    while (result.frames_submitted < total_frames) {
      if (cancel_requested()) {
        sink.stop();

        result.outcome = PlaybackOutcome::Cancelled;

        microphone_gate_.end_playback();
        return result;
      }

      const std::size_t remaining_frames =
        total_frames - result.frames_submitted;

      const std::size_t frames_this_chunk =
        std::min(chunk_frames_, remaining_frames);

      const std::size_t sample_offset =
        result.frames_submitted * channel_count;

      const std::size_t samples_this_chunk =
        frames_this_chunk * channel_count;

      sink.write(
        samples.subspan(
          sample_offset,
          samples_this_chunk));

      result.frames_submitted += frames_this_chunk;
      ++result.chunks_submitted;
    }

    if (cancel_requested()) {
      sink.stop();
      result.outcome = PlaybackOutcome::Cancelled;
    } else {
      sink.drain();
      result.outcome = PlaybackOutcome::Completed;
    }

    microphone_gate_.end_playback();
    return result;
  } catch (...) {
    sink.stop();
    microphone_gate_.end_playback();
    throw;
  }
}

void PlaybackController::request_cancel() noexcept
{
  cancel_requested_.store(
    true,
    std::memory_order_release);
}

void PlaybackController::clear_cancel() noexcept
{
  cancel_requested_.store(
    false,
    std::memory_order_release);
}

bool PlaybackController::cancel_requested() const noexcept
{
  return cancel_requested_.load(
    std::memory_order_acquire);
}

std::size_t PlaybackController::chunk_frames() const noexcept
{
  return chunk_frames_;
}

}  // namespace savo_speech::audio
