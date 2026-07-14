#ifndef SAVO_SPEECH__SESSION__SPEECH_PHASE_HPP_
#define SAVO_SPEECH__SESSION__SPEECH_PHASE_HPP_

#include <cstdint>
#include <string_view>

namespace savo_speech::session
{

enum class SpeechPhase : std::uint8_t
{
  Starting = 0U,
  WaitingForAudio = 1U,
  Idle = 2U,
  WakeWordDetected = 3U,
  Listening = 4U,
  Recording = 5U,
  Processing = 6U,
  Speaking = 7U,
  Muted = 8U,
  Canceling = 9U,
  Recovering = 10U,
  Error = 11U,
  ShuttingDown = 12U,
  Disabled = 13U
};

[[nodiscard]] constexpr std::string_view to_string(
  const SpeechPhase phase) noexcept
{
  switch (phase) {
    case SpeechPhase::Starting:
      return "starting";

    case SpeechPhase::WaitingForAudio:
      return "waiting_for_audio";

    case SpeechPhase::Idle:
      return "idle";

    case SpeechPhase::WakeWordDetected:
      return "wake_word_detected";

    case SpeechPhase::Listening:
      return "listening";

    case SpeechPhase::Recording:
      return "recording";

    case SpeechPhase::Processing:
      return "processing";

    case SpeechPhase::Speaking:
      return "speaking";

    case SpeechPhase::Muted:
      return "muted";

    case SpeechPhase::Canceling:
      return "canceling";

    case SpeechPhase::Recovering:
      return "recovering";

    case SpeechPhase::Error:
      return "error";

    case SpeechPhase::ShuttingDown:
      return "shutting_down";

    case SpeechPhase::Disabled:
      return "disabled";
  }

  return "unknown";
}

[[nodiscard]] constexpr bool is_active_phase(
  const SpeechPhase phase) noexcept
{
  switch (phase) {
    case SpeechPhase::WakeWordDetected:
    case SpeechPhase::Listening:
    case SpeechPhase::Recording:
    case SpeechPhase::Processing:
    case SpeechPhase::Speaking:
    case SpeechPhase::Canceling:
    case SpeechPhase::Recovering:
      return true;

    case SpeechPhase::Starting:
    case SpeechPhase::WaitingForAudio:
    case SpeechPhase::Idle:
    case SpeechPhase::Muted:
    case SpeechPhase::Error:
    case SpeechPhase::ShuttingDown:
    case SpeechPhase::Disabled:
      return false;
  }

  return false;
}

[[nodiscard]] constexpr bool is_terminal_phase(
  const SpeechPhase phase) noexcept
{
  return
    phase == SpeechPhase::Idle ||
    phase == SpeechPhase::Error ||
    phase == SpeechPhase::Disabled ||
    phase == SpeechPhase::ShuttingDown;
}

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__SPEECH_PHASE_HPP_
