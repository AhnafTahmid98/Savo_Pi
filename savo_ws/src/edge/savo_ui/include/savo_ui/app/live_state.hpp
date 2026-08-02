// Copyright 2026 Ahnaf Tahmid
#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <string_view>

namespace savo_ui
{

enum class SpeechLifecycle
{
  Waiting,
  Listening,
  Recording,
  Sending,
  Thinking,
  Playing,
  Completed,
  Failed,
  Canceled,
};

[[nodiscard]] SpeechLifecycle classify_speech_lifecycle(std::string_view text);
[[nodiscard]] std::string lifecycle_label(SpeechLifecycle lifecycle);
[[nodiscard]] std::string bounded_ui_text(std::string_view text, std::size_t maximum_bytes);
[[nodiscard]] std::string status_field(std::string_view text, std::string_view key);
[[nodiscard]] bool feed_is_stale(
  bool seen,
  std::chrono::steady_clock::time_point updated,
  std::chrono::steady_clock::time_point now,
  std::chrono::milliseconds timeout);

}  // namespace savo_ui
