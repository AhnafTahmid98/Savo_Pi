#pragma once

#include <array>
#include <string>
#include <string_view>

namespace savo_speech::topics
{

inline constexpr std::string_view STATE = "/savo_speech/state";
inline constexpr std::string_view STATUS = "/savo_speech/status";
inline constexpr std::string_view HEALTH = "/savo_speech/health";
inline constexpr std::string_view HEARTBEAT = "/savo_speech/heartbeat";

inline constexpr std::string_view WAKE_STATE = "/savo_speech/wake_state";
inline constexpr std::string_view WAKE_WORD_DETECTED = "/savo_speech/wake_word/detected";
inline constexpr std::string_view LISTENING = "/savo_speech/listening";
inline constexpr std::string_view INPUT_MUTED = "/savo_speech/input_muted";
inline constexpr std::string_view OUTPUT_MUTED = "/savo_speech/output_muted";
inline constexpr std::string_view TRANSCRIPT = "/savo_speech/transcript";

inline constexpr std::string_view TTS_GATE = "/savo_speech/tts_gate";
inline constexpr std::string_view TTS_STARTED = "/savo_speech/tts/started";
inline constexpr std::string_view TTS_FINISHED = "/savo_speech/tts/finished";
inline constexpr std::string_view TTS_CANCELLED = "/savo_speech/tts/cancelled";

inline constexpr std::string_view FACE_STATE = "/savo_speech/face_state";
inline constexpr std::string_view LAST_ERROR = "/savo_speech/last_error";

inline constexpr std::string_view SERVICE_WAKE = "/savo_speech/wake";
inline constexpr std::string_view SERVICE_SLEEP = "/savo_speech/sleep";
inline constexpr std::string_view SERVICE_START_LISTENING =
  "/savo_speech/start_listening";
inline constexpr std::string_view SERVICE_STOP_LISTENING =
  "/savo_speech/stop_listening";
inline constexpr std::string_view SERVICE_CANCEL = "/savo_speech/cancel";
inline constexpr std::string_view SERVICE_MUTE_INPUT = "/savo_speech/mute_input";
inline constexpr std::string_view SERVICE_MUTE_OUTPUT = "/savo_speech/mute_output";
inline constexpr std::string_view SERVICE_RELOAD_AUDIO_DEVICE =
  "/savo_speech/reload_audio_device";

inline constexpr std::array<std::string_view, 16> PUBLIC_TOPICS = {
  STATE,
  STATUS,
  HEALTH,
  HEARTBEAT,
  WAKE_STATE,
  WAKE_WORD_DETECTED,
  LISTENING,
  INPUT_MUTED,
  OUTPUT_MUTED,
  TRANSCRIPT,
  TTS_GATE,
  TTS_STARTED,
  TTS_FINISHED,
  TTS_CANCELLED,
  FACE_STATE,
  LAST_ERROR,
};

inline constexpr std::array<std::string_view, 8> PUBLIC_SERVICES = {
  SERVICE_WAKE,
  SERVICE_SLEEP,
  SERVICE_START_LISTENING,
  SERVICE_STOP_LISTENING,
  SERVICE_CANCEL,
  SERVICE_MUTE_INPUT,
  SERVICE_MUTE_OUTPUT,
  SERVICE_RELOAD_AUDIO_DEVICE,
};

inline std::string join_topic(
  const std::string_view prefix,
  const std::string_view suffix)
{
  if (prefix.empty()) {
    return std::string(suffix);
  }

  if (suffix.empty()) {
    return std::string(prefix);
  }

  if (prefix.back() == '/' && suffix.front() == '/') {
    return std::string(prefix) + std::string(suffix.substr(1));
  }

  if (prefix.back() != '/' && suffix.front() != '/') {
    return std::string(prefix) + "/" + std::string(suffix);
  }

  return std::string(prefix) + std::string(suffix);
}

constexpr bool is_state_topic(const std::string_view topic) noexcept
{
  return topic == STATE ||
         topic == WAKE_STATE ||
         topic == LISTENING ||
         topic == INPUT_MUTED ||
         topic == OUTPUT_MUTED ||
         topic == TTS_GATE ||
         topic == FACE_STATE ||
         topic == LAST_ERROR;
}

constexpr bool is_event_topic(const std::string_view topic) noexcept
{
  return topic == WAKE_WORD_DETECTED ||
         topic == TRANSCRIPT ||
         topic == TTS_STARTED ||
         topic == TTS_FINISHED ||
         topic == TTS_CANCELLED;
}

constexpr bool is_status_topic(const std::string_view topic) noexcept
{
  return topic == STATUS ||
         topic == HEALTH ||
         topic == HEARTBEAT;
}

constexpr bool is_public_topic(const std::string_view topic) noexcept
{
  for (const auto candidate : PUBLIC_TOPICS) {
    if (candidate == topic) {
      return true;
    }
  }

  return false;
}

constexpr bool is_public_service(const std::string_view service) noexcept
{
  for (const auto candidate : PUBLIC_SERVICES) {
    if (candidate == service) {
      return true;
    }
  }

  return false;
}

}  // namespace savo_speech::topics
