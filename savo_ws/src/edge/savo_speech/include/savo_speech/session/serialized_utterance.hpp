// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__SESSION__SERIALIZED_UTTERANCE_HPP_
#define SAVO_SPEECH__SESSION__SERIALIZED_UTTERANCE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "savo_speech/audio/audio_format.hpp"
#include "savo_speech/audio/audio_frame.hpp"
#include "savo_speech/session/utterance_session_event.hpp"

namespace savo_speech::session
{

struct SerializedUtterance
{
  using Clock = audio::AudioFrame::Clock;

  std::uint64_t utterance_id{0U};

  std::uint64_t wake_event_id{0U};
  std::uint64_t wake_frame_sequence{0U};

  Clock::time_point wake_detected_at{};

  std::string wake_phrase{};
  double wake_confidence{0.0};

  std::uint64_t vad_segment_id{0U};

  std::uint64_t speech_start_frame_sequence{0U};
  std::uint64_t speech_end_frame_sequence{0U};

  Clock::time_point speech_started_at{};
  Clock::time_point speech_ended_at{};

  std::uint64_t first_audio_frame_sequence{0U};
  std::uint64_t last_audio_frame_sequence{0U};

  Clock::time_point audio_started_at{};
  Clock::time_point audio_ended_at{};

  Clock::time_point completed_at{};

  UtteranceCompletionReason completion_reason{
    UtteranceCompletionReason::SpeechEnded};

  audio::AudioFormat audio_format{};

  std::size_t sample_count{0U};
  std::size_t pre_roll_samples{0U};

  std::uint64_t sequence_gap_count{0U};
  std::uint64_t missing_audio_frames{0U};

  std::vector<std::uint8_t> wav_bytes{};

  [[nodiscard]] bool is_valid() const noexcept
  {
    if (
      utterance_id == 0U ||
      !audio_format.is_valid() ||
      audio_format.sample_format !=
      audio::PcmSampleFormat::Signed16LittleEndian ||
      sample_count == 0U ||
      pre_roll_samples > sample_count ||
      audio_format.channels == 0U ||
      sample_count %
      static_cast<std::size_t>(audio_format.channels) != 0U)
    {
      return false;
    }

    constexpr std::size_t kWavHeaderBytes{44U};
    constexpr std::size_t kBytesPerSample{2U};

    if (
      sample_count >
      (static_cast<std::size_t>(-1) - kWavHeaderBytes) /
      kBytesPerSample)
    {
      return false;
    }

    const std::size_t expected_size =
      kWavHeaderBytes + sample_count * kBytesPerSample;

    if (wav_bytes.size() != expected_size) {
      return false;
    }

    if (wav_bytes.size() < kWavHeaderBytes) {
      return false;
    }

    return
      wav_bytes[0U] == static_cast<std::uint8_t>('R') &&
      wav_bytes[1U] == static_cast<std::uint8_t>('I') &&
      wav_bytes[2U] == static_cast<std::uint8_t>('F') &&
      wav_bytes[3U] == static_cast<std::uint8_t>('F') &&
      wav_bytes[8U] == static_cast<std::uint8_t>('W') &&
      wav_bytes[9U] == static_cast<std::uint8_t>('A') &&
      wav_bytes[10U] == static_cast<std::uint8_t>('V') &&
      wav_bytes[11U] == static_cast<std::uint8_t>('E');
  }
};

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__SERIALIZED_UTTERANCE_HPP_
