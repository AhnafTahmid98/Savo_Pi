// Copyright 2026 Ahnaf Tahmid
#include <algorithm>
#include <cctype>

#include "savo_ui/app/live_state.hpp"

namespace savo_ui
{
namespace
{

std::string lowercase(std::string_view text)
{
  std::string output{text};
  std::transform(output.begin(), output.end(), output.begin(), [](const unsigned char value) {
      return static_cast<char>(std::tolower(value));
  });
  return output;
}

}  // namespace

SpeechLifecycle classify_speech_lifecycle(const std::string_view text)
{
  const auto value = lowercase(text);
  if (value.find("cancel") != std::string::npos) {return SpeechLifecycle::Canceled;}
  if (value.find("fail") != std::string::npos || value.find("error") != std::string::npos) {
    return SpeechLifecycle::Failed;
  }
  if (value.find("play") != std::string::npos || value.find("speak") != std::string::npos) {
    return SpeechLifecycle::Playing;
  }
  if (value.find("think") != std::string::npos || value.find("synth") != std::string::npos ||
    value.find("transcrib") != std::string::npos) {return SpeechLifecycle::Thinking;}
  if (value.find("send") != std::string::npos) {return SpeechLifecycle::Sending;}
  if (value.find("record") != std::string::npos) {return SpeechLifecycle::Recording;}
  if (value.find("listen") != std::string::npos) {return SpeechLifecycle::Listening;}
  if (value.find("complete") != std::string::npos) {return SpeechLifecycle::Completed;}
  return SpeechLifecycle::Waiting;
}

std::string lifecycle_label(const SpeechLifecycle lifecycle)
{
  switch (lifecycle) {
    case SpeechLifecycle::Waiting: return "WAITING";
    case SpeechLifecycle::Listening: return "LISTENING";
    case SpeechLifecycle::Recording: return "RECORDING";
    case SpeechLifecycle::Sending: return "SENDING";
    case SpeechLifecycle::Thinking: return "THINKING";
    case SpeechLifecycle::Playing: return "PLAYING";
    case SpeechLifecycle::Completed: return "COMPLETED";
    case SpeechLifecycle::Failed: return "FAILED";
    case SpeechLifecycle::Canceled: return "CANCELED";
  }
  return "WAITING";
}

std::string bounded_ui_text(const std::string_view text, const std::size_t maximum_bytes)
{
  if (maximum_bytes == 0U) {return {};}
  if (text.size() <= maximum_bytes) {return std::string{text};}
  if (maximum_bytes <= 3U) {return std::string{text.substr(0U, maximum_bytes)};}
  return std::string{text.substr(0U, maximum_bytes - 3U)} + "...";
}

std::string status_field(const std::string_view text, const std::string_view key)
{
  const std::string token = std::string{key} + "=";
  const auto position = text.find(token);
  if (position == std::string_view::npos) {return {};}
  const auto start = position + token.size();
  const auto end = text.find_first_of(";, \t\r\n}", start);
  return std::string{text.substr(start,
        end == std::string_view::npos ? text.size() - start : end - start)};
}

bool feed_is_stale(
  const bool seen,
  const std::chrono::steady_clock::time_point updated,
  const std::chrono::steady_clock::time_point now,
  const std::chrono::milliseconds timeout)
{
  return !seen || now < updated || now - updated > timeout;
}

}  // namespace savo_ui
