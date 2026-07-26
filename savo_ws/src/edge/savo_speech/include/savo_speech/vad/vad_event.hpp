#ifndef SAVO_SPEECH__VAD__VAD_EVENT_HPP_
#define SAVO_SPEECH__VAD__VAD_EVENT_HPP_

#include <chrono>
#include <cstdint>
#include <string_view>

#include "savo_speech/audio/audio_frame.hpp"

namespace savo_speech::vad
{

enum class VadState : std::uint8_t
{
  Silence = 0U,
  Speech
};

[[nodiscard]] constexpr std::string_view to_string(
  const VadState state) noexcept
{
  switch (state) {
    case VadState::Silence:
      return "silence";

    case VadState::Speech:
      return "speech";
  }

  return "unknown";
}

enum class VadEventType : std::uint8_t
{
  SpeechStarted = 0U,
  SpeechEnded
};

[[nodiscard]] constexpr std::string_view to_string(
  const VadEventType type) noexcept
{
  switch (type) {
    case VadEventType::SpeechStarted:
      return "speech_started";

    case VadEventType::SpeechEnded:
      return "speech_ended";
  }

  return "unknown";
}

struct VadEvent
{
  using Clock = audio::AudioFrame::Clock;

  std::uint64_t event_id{0U};
  std::uint64_t segment_id{0U};

  std::uint64_t frame_sequence{0U};
  Clock::time_point occurred_at{};

  VadEventType type{VadEventType::SpeechStarted};

  double speech_score{0.0};
};

}  // namespace savo_speech::vad

#endif  // SAVO_SPEECH__VAD__VAD_EVENT_HPP_
