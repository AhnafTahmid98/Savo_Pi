#ifndef SAVO_SPEECH__SESSION__SPEECH_ERROR_HPP_
#define SAVO_SPEECH__SESSION__SPEECH_ERROR_HPP_

#include <cstdint>
#include <string_view>

namespace savo_speech::session
{

enum class SpeechError : std::uint8_t
{
  None = 0U,
  AudioNotInitialized = 1U,
  InvalidConfiguration = 2U,
  CaptureDeviceUnavailable = 3U,
  CaptureStreamFailed = 4U,
  PlaybackDeviceUnavailable = 5U,
  PlaybackStreamFailed = 6U,
  NoSpeechTimeout = 7U,
  UtteranceTooLong = 8U,
  WavEncodingFailed = 9U,
  SavoMindUnavailable = 10U,
  SavoMindTimeout = 11U,
  SavoMindInvalidResponse = 12U,
  SttFailed = 13U,
  LlmFailed = 14U,
  TtsFailed = 15U,
  PlaybackFailed = 16U,
  SessionCanceled = 17U,
  InternalError = 18U
};

[[nodiscard]] constexpr std::string_view to_string(
  const SpeechError error) noexcept
{
  switch (error) {
    case SpeechError::None:
      return "none";

    case SpeechError::AudioNotInitialized:
      return "audio_not_initialized";

    case SpeechError::InvalidConfiguration:
      return "invalid_configuration";

    case SpeechError::CaptureDeviceUnavailable:
      return "capture_device_unavailable";

    case SpeechError::CaptureStreamFailed:
      return "capture_stream_failed";

    case SpeechError::PlaybackDeviceUnavailable:
      return "playback_device_unavailable";

    case SpeechError::PlaybackStreamFailed:
      return "playback_stream_failed";

    case SpeechError::NoSpeechTimeout:
      return "no_speech_timeout";

    case SpeechError::UtteranceTooLong:
      return "utterance_too_long";

    case SpeechError::WavEncodingFailed:
      return "wav_encoding_failed";

    case SpeechError::SavoMindUnavailable:
      return "savomind_unavailable";

    case SpeechError::SavoMindTimeout:
      return "savomind_timeout";

    case SpeechError::SavoMindInvalidResponse:
      return "savomind_invalid_response";

    case SpeechError::SttFailed:
      return "stt_failed";

    case SpeechError::LlmFailed:
      return "llm_failed";

    case SpeechError::TtsFailed:
      return "tts_failed";

    case SpeechError::PlaybackFailed:
      return "playback_failed";

    case SpeechError::SessionCanceled:
      return "session_canceled";

    case SpeechError::InternalError:
      return "internal_error";
  }

  return "unknown";
}

[[nodiscard]] constexpr bool is_error(
  const SpeechError error) noexcept
{
  return error != SpeechError::None;
}

[[nodiscard]] constexpr bool is_recoverable(
  const SpeechError error) noexcept
{
  switch (error) {
    case SpeechError::AudioNotInitialized:
    case SpeechError::CaptureDeviceUnavailable:
    case SpeechError::CaptureStreamFailed:
    case SpeechError::PlaybackDeviceUnavailable:
    case SpeechError::PlaybackStreamFailed:
    case SpeechError::NoSpeechTimeout:
    case SpeechError::SavoMindUnavailable:
    case SpeechError::SavoMindTimeout:
    case SpeechError::PlaybackFailed:
    case SpeechError::SessionCanceled:
      return true;

    case SpeechError::None:
    case SpeechError::InvalidConfiguration:
    case SpeechError::UtteranceTooLong:
    case SpeechError::WavEncodingFailed:
    case SpeechError::SavoMindInvalidResponse:
    case SpeechError::SttFailed:
    case SpeechError::LlmFailed:
    case SpeechError::TtsFailed:
    case SpeechError::InternalError:
      return false;
  }

  return false;
}

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__SPEECH_ERROR_HPP_
