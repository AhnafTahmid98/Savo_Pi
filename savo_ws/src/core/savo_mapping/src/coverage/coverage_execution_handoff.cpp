#include "savo_mapping/coverage_execution_handoff.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace savo_mapping::coverage
{
namespace
{

bool text_is_blank(const std::string & value)
{
  return value.empty() ||
         std::all_of(
    value.begin(),
    value.end(),
    [](const unsigned char character) {
      return std::isspace(character) != 0;
    });
}

bool valid_mission_prefix(const std::string & value)
{
  if (text_is_blank(value) || value.size() > 32U) {
    return false;
  }

  return std::all_of(
    value.begin(),
    value.end(),
    [](const unsigned char character) {
      return std::isalnum(character) != 0 ||
             character == '-' ||
             character == '_';
    });
}

}  // namespace

std::string validate_coverage_execution_handoff_policy(
  const CoverageExecutionHandoffPolicy & policy)
{
  if (!std::isfinite(policy.server_wait_timeout_sec) ||
    policy.server_wait_timeout_sec <= 0.0)
  {
    return "coverage_handoff_server_wait_timeout_invalid";
  }

  if (!std::isfinite(policy.goal_response_timeout_sec) ||
    policy.goal_response_timeout_sec <= 0.0)
  {
    return "coverage_handoff_goal_response_timeout_invalid";
  }

  if (!std::isfinite(policy.cancel_timeout_sec) ||
    policy.cancel_timeout_sec <= 0.0)
  {
    return "coverage_handoff_cancel_timeout_invalid";
  }

  if (!std::isfinite(policy.execution_timeout_sec) ||
    policy.execution_timeout_sec < 0.0)
  {
    return "coverage_handoff_execution_timeout_invalid";
  }

  if (policy.maximum_waypoints == 0U) {
    return "coverage_handoff_maximum_waypoints_invalid";
  }

  return {};
}

std::string make_coverage_mission_id(
  const std::string & prefix,
  const std::uint64_t candidate_generation,
  const std::int64_t event_time_ns)
{
  if (!valid_mission_prefix(prefix)) {
    return {};
  }

  if (candidate_generation == 0U || event_time_ns < 0) {
    return {};
  }

  std::ostringstream output;
  output
    << prefix
    << '-'
    << candidate_generation
    << '-'
    << event_time_ns;
  return output.str();
}

}  // namespace savo_mapping::coverage
