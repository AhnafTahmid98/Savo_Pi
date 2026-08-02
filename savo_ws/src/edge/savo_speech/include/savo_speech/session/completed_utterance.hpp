// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_HPP_
#define SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/audio_frame.hpp"
#include "savo_speech/session/utterance_session_event.hpp"

namespace savo_speech::session
{

struct CompletedUtterance
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

  audio::AudioBuffer audio{};

  std::size_t pre_roll_samples{0U};

  std::uint64_t sequence_gap_count{0U};
  std::uint64_t missing_audio_frames{0U};
};

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_HPP_
