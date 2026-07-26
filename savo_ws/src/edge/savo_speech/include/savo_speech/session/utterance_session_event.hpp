#ifndef SAVO_SPEECH__SESSION__UTTERANCE_SESSION_EVENT_HPP_
#define SAVO_SPEECH__SESSION__UTTERANCE_SESSION_EVENT_HPP_

#include <cstdint>
#include <string_view>

namespace savo_speech::session
{

enum class UtteranceSessionState : std::uint8_t
{
  Idle = 0U,
  Armed,
  Recording
};

[[nodiscard]] constexpr std::string_view to_string(
  const UtteranceSessionState state) noexcept
{
  switch (state) {
    case UtteranceSessionState::Idle:
      return "idle";

    case UtteranceSessionState::Armed:
      return "armed";

    case UtteranceSessionState::Recording:
      return "recording";
  }

  return "unknown";
}

enum class UtteranceCompletionReason : std::uint8_t
{
  SpeechEnded = 0U,
  MaximumDurationReached
};

[[nodiscard]] constexpr std::string_view to_string(
  const UtteranceCompletionReason reason) noexcept
{
  switch (reason) {
    case UtteranceCompletionReason::SpeechEnded:
      return "speech_ended";

    case UtteranceCompletionReason::MaximumDurationReached:
      return "maximum_duration_reached";
  }

  return "unknown";
}

enum class UtteranceCancellationReason : std::uint8_t
{
  None = 0U,
  SpeechStartTimeout,
  ExplicitCancellation,
  NoAudioCaptured
};

[[nodiscard]] constexpr std::string_view to_string(
  const UtteranceCancellationReason reason) noexcept
{
  switch (reason) {
    case UtteranceCancellationReason::None:
      return "none";

    case UtteranceCancellationReason::SpeechStartTimeout:
      return "speech_start_timeout";

    case UtteranceCancellationReason::ExplicitCancellation:
      return "explicit_cancellation";

    case UtteranceCancellationReason::NoAudioCaptured:
      return "no_audio_captured";
  }

  return "unknown";
}

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__UTTERANCE_SESSION_EVENT_HPP_
