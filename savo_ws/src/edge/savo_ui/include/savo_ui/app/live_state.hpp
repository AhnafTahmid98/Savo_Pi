// Copyright 2026 Ahnaf Tahmid
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
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

[[nodiscard]] std::string mapping_summary(
  std::string_view state_text,
  bool active,
  double coverage_completion_ratio,
  std::string_view scan360_stage,
  std::string_view scan360_state,
  std::uint32_t pending_candidate_count,
  bool approval_pending,
  std::string_view release_state,
  std::string_view release_id,
  std::string_view reason);

[[nodiscard]] std::string location_event_summary(
  std::uint8_t event_type,
  std::string_view candidate_id,
  std::string_view location_id,
  std::string_view reason);

[[nodiscard]] bool feed_is_stale(
  bool seen,
  std::chrono::steady_clock::time_point updated,
  std::chrono::steady_clock::time_point now,
  std::chrono::milliseconds timeout);

}  // namespace savo_ui
