// Copyright 2026 Ahnaf Tahmid
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>

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

std::string mapping_summary(
  const std::string_view state_text,
  const bool active,
  const double coverage_completion_ratio,
  const std::string_view scan360_stage,
  const std::string_view scan360_state,
  const std::uint32_t pending_candidate_count,
  const bool approval_pending,
  const std::string_view release_state,
  const std::string_view release_id,
  const std::string_view reason)
{
  std::ostringstream output;
  output << (state_text.empty() ? (active ? "ACTIVE" : "IDLE") : std::string{state_text});
  if (std::isfinite(coverage_completion_ratio) && coverage_completion_ratio > 0.0) {
    output << " coverage=" << std::fixed << std::setprecision(0)
           << std::clamp(coverage_completion_ratio * 100.0, 0.0, 100.0) << '%';
  }
  if (!scan360_state.empty() && scan360_state != "idle") {
    output << " scan360=";
    if (!scan360_stage.empty()) {output << scan360_stage << ':';}
    output << scan360_state;
  }
  if (pending_candidate_count > 0U) {output << " pending_tags=" << pending_candidate_count;}
  if (approval_pending) {output << " approval=required";}
  if (!release_state.empty()) {
    output << " release=" << release_state;
    if (!release_id.empty()) {output << ':' << release_id;}
  }
  if (!reason.empty()) {output << " reason=" << reason;}
  return bounded_ui_text(output.str(), 220U);
}

std::string location_event_summary(
  const std::uint8_t event_type,
  const std::string_view candidate_id,
  const std::string_view location_id,
  const std::string_view reason)
{
  const char * label = "location_event";
  switch (event_type) {
    case 1U: label = "candidate_registered"; break;
    case 2U: label = "location_approved"; break;
    case 3U: label = "candidate_rejected"; break;
    case 4U: label = "location_updated"; break;
    case 5U: label = "location_enabled"; break;
    case 6U: label = "location_disabled"; break;
    case 7U: label = "location_retired"; break;
    case 8U: label = "import_completed"; break;
    case 9U: label = "storage_degraded"; break;
    case 10U: label = "release_prepared"; break;
    case 11U: label = "release_committed"; break;
    case 12U: label = "release_rolled_back"; break;
    default: break;
  }
  std::ostringstream output;
  output << label;
  if (!candidate_id.empty()) {output << " candidate=" << candidate_id;}
  if (!location_id.empty()) {output << " location=" << location_id;}
  if (!reason.empty()) {output << " reason=" << reason;}
  return bounded_ui_text(output.str(), 160U);
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
