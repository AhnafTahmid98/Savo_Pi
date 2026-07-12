#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <string>
#include <string_view>

namespace savo_speech
{

enum class SpeechState : std::uint8_t
{
  STARTING = 0,
  SLEEPING,
  WAKE_DETECTED,
  AWAKE_IDLE,
  LISTENING,
  UTTERANCE_READY,
  TRANSCRIBING,
  THINKING,
  SYNTHESIZING,
  SPEAKING,
  ERROR,
  COUNT,
};

inline constexpr std::array<std::string_view, static_cast<std::size_t>(SpeechState::COUNT)>
SPEECH_STATE_NAMES = {
  "STARTING",
  "SLEEPING",
  "WAKE_DETECTED",
  "AWAKE_IDLE",
  "LISTENING",
  "UTTERANCE_READY",
  "TRANSCRIBING",
  "THINKING",
  "SYNTHESIZING",
  "SPEAKING",
  "ERROR",
};

enum class SpeechEvent : std::uint8_t
{
  STARTUP_COMPLETE = 0,
  WAKE_DETECTED,
  LISTEN_REQUESTED,
  SPEECH_STARTED,
  UTTERANCE_READY,
  STT_STARTED,
  TRANSCRIPT_READY,
  THINKING_STARTED,
  REPLY_READY,
  TTS_STARTED,
  AUDIO_READY,
  PLAYBACK_STARTED,
  PLAYBACK_FINISHED,
  CANCEL_REQUESTED,
  IDLE_TIMEOUT,
  MUTE_ENABLED,
  MUTE_DISABLED,
  FAILURE,
  RESET,
  COUNT,
};

inline constexpr std::array<std::string_view, static_cast<std::size_t>(SpeechEvent::COUNT)>
SPEECH_EVENT_NAMES = {
  "STARTUP_COMPLETE",
  "WAKE_DETECTED",
  "LISTEN_REQUESTED",
  "SPEECH_STARTED",
  "UTTERANCE_READY",
  "STT_STARTED",
  "TRANSCRIPT_READY",
  "THINKING_STARTED",
  "REPLY_READY",
  "TTS_STARTED",
  "AUDIO_READY",
  "PLAYBACK_STARTED",
  "PLAYBACK_FINISHED",
  "CANCEL_REQUESTED",
  "IDLE_TIMEOUT",
  "MUTE_ENABLED",
  "MUTE_DISABLED",
  "FAILURE",
  "RESET",
};

enum class WakeState : std::uint8_t
{
  DISABLED = 0,
  SLEEPING,
  AWAKE,
  COOLDOWN,
  ERROR,
  COUNT,
};

inline constexpr std::array<std::string_view, static_cast<std::size_t>(WakeState::COUNT)>
WAKE_STATE_NAMES = {
  "DISABLED",
  "SLEEPING",
  "AWAKE",
  "COOLDOWN",
  "ERROR",
};

enum class ProviderState : std::uint8_t
{
  DISABLED = 0,
  UNAVAILABLE,
  STARTING,
  READY,
  BUSY,
  DEGRADED,
  ERROR,
  COUNT,
};

inline constexpr std::array<std::string_view, static_cast<std::size_t>(ProviderState::COUNT)>
PROVIDER_STATE_NAMES = {
  "DISABLED",
  "UNAVAILABLE",
  "STARTING",
  "READY",
  "BUSY",
  "DEGRADED",
  "ERROR",
};

namespace detail
{

inline std::string normalize_token(std::string_view value)
{
  std::string normalized;
  normalized.reserve(value.size());

  for (const char character : value) {
    const auto byte = static_cast<unsigned char>(character);

    if (character == '-' || std::isspace(byte) != 0) {
      normalized.push_back('_');
    } else {
      normalized.push_back(static_cast<char>(std::toupper(byte)));
    }
  }

  return normalized;
}

template<typename EnumT, std::size_t SizeT>
constexpr std::string_view enum_to_string(
  const EnumT value,
  const std::array<std::string_view, SizeT> & names,
  const std::string_view fallback = "UNKNOWN") noexcept
{
  const auto index = static_cast<std::size_t>(value);
  return index < names.size() ? names[index] : fallback;
}

template<typename EnumT, std::size_t SizeT>
inline EnumT enum_from_string(
  const std::string_view value,
  const std::array<std::string_view, SizeT> & names,
  const EnumT fallback)
{
  const std::string normalized = normalize_token(value);

  const auto iterator = std::find(names.begin(), names.end(), normalized);
  if (iterator == names.end()) {
    return fallback;
  }

  return static_cast<EnumT>(std::distance(names.begin(), iterator));
}

}  // namespace detail

constexpr std::string_view to_string(const SpeechState value) noexcept
{
  return detail::enum_to_string(value, SPEECH_STATE_NAMES);
}

constexpr std::string_view to_string(const SpeechEvent value) noexcept
{
  return detail::enum_to_string(value, SPEECH_EVENT_NAMES);
}

constexpr std::string_view to_string(const WakeState value) noexcept
{
  return detail::enum_to_string(value, WAKE_STATE_NAMES);
}

constexpr std::string_view to_string(const ProviderState value) noexcept
{
  return detail::enum_to_string(value, PROVIDER_STATE_NAMES);
}

inline SpeechState speech_state_from_string(
  const std::string_view value,
  const SpeechState fallback = SpeechState::ERROR)
{
  return detail::enum_from_string(value, SPEECH_STATE_NAMES, fallback);
}

inline SpeechEvent speech_event_from_string(
  const std::string_view value,
  const SpeechEvent fallback = SpeechEvent::FAILURE)
{
  return detail::enum_from_string(value, SPEECH_EVENT_NAMES, fallback);
}

inline WakeState wake_state_from_string(
  const std::string_view value,
  const WakeState fallback = WakeState::ERROR)
{
  return detail::enum_from_string(value, WAKE_STATE_NAMES, fallback);
}

inline ProviderState provider_state_from_string(
  const std::string_view value,
  const ProviderState fallback = ProviderState::ERROR)
{
  return detail::enum_from_string(value, PROVIDER_STATE_NAMES, fallback);
}

constexpr bool is_busy_state(const SpeechState state) noexcept
{
  return state == SpeechState::WAKE_DETECTED ||
         state == SpeechState::LISTENING ||
         state == SpeechState::UTTERANCE_READY ||
         state == SpeechState::TRANSCRIBING ||
         state == SpeechState::THINKING ||
         state == SpeechState::SYNTHESIZING ||
         state == SpeechState::SPEAKING;
}

constexpr bool is_capture_state(const SpeechState state) noexcept
{
  return state == SpeechState::LISTENING ||
         state == SpeechState::UTTERANCE_READY;
}

constexpr bool is_output_state(const SpeechState state) noexcept
{
  return state == SpeechState::SYNTHESIZING ||
         state == SpeechState::SPEAKING;
}

}  // namespace savo_speech
