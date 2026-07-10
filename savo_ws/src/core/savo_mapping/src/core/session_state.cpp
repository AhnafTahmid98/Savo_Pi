#include "savo_mapping/session_state.hpp"

namespace savo_mapping
{

std::string_view to_string(SessionState state)
{
  switch (state) {
    case SessionState::Idle:
      return "idle";
    case SessionState::Starting:
      return "starting";
    case SessionState::Active:
      return "active";
    case SessionState::Paused:
      return "paused";
    case SessionState::Saving:
      return "saving";
    case SessionState::Saved:
      return "saved";
    case SessionState::Cancelled:
      return "cancelled";
    case SessionState::Failed:
      return "failed";
  }

  return "unknown";
}

std::optional<SessionState> session_state_from_string(std::string_view text)
{
  if (text == "idle") {
    return SessionState::Idle;
  }
  if (text == "starting") {
    return SessionState::Starting;
  }
  if (text == "active") {
    return SessionState::Active;
  }
  if (text == "paused") {
    return SessionState::Paused;
  }
  if (text == "saving") {
    return SessionState::Saving;
  }
  if (text == "saved") {
    return SessionState::Saved;
  }
  if (text == "cancelled") {
    return SessionState::Cancelled;
  }
  if (text == "failed") {
    return SessionState::Failed;
  }

  return std::nullopt;
}

bool is_session_running(SessionState state)
{
  return state == SessionState::Starting ||
         state == SessionState::Active ||
         state == SessionState::Paused ||
         state == SessionState::Saving;
}

bool is_session_finished(SessionState state)
{
  return state == SessionState::Saved ||
         state == SessionState::Cancelled ||
         state == SessionState::Failed;
}

bool is_successful_session(SessionState state)
{
  return state == SessionState::Saved;
}

bool can_accept_mapping_data(SessionState state)
{
  return state == SessionState::Active ||
         state == SessionState::Paused ||
         state == SessionState::Saving;
}

bool can_request_save(SessionState state)
{
  return state == SessionState::Active ||
         state == SessionState::Paused;
}

bool can_cancel_session(SessionState state)
{
  return state == SessionState::Starting ||
         state == SessionState::Active ||
         state == SessionState::Paused;
}

}  // namespace savo_mapping
