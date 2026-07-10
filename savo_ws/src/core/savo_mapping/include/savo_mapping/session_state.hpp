#pragma once

#include <optional>
#include <string_view>

namespace savo_mapping
{

enum class SessionState
{
  Idle,
  Starting,
  Active,
  Paused,
  Saving,
  Saved,
  Cancelled,
  Failed
};

std::string_view to_string(SessionState state);
std::optional<SessionState> session_state_from_string(std::string_view text);

bool is_session_running(SessionState state);
bool is_session_finished(SessionState state);
bool is_successful_session(SessionState state);
bool can_accept_mapping_data(SessionState state);
bool can_request_save(SessionState state);
bool can_cancel_session(SessionState state);

}  // namespace savo_mapping
