#pragma once

#include <cstdint>
#include <string>

namespace savo_mapping::coverage
{

inline constexpr char kCoverageExecutionActionName[] =
  "/savo_nav/coverage/execute_path";

inline constexpr char kCoveragePlanTopic[] =
  "/savo_mapping/coverage/path";

inline constexpr char kCoverageExecutionStateTopic[] =
  "/savo_mapping/coverage_execution/state";

inline constexpr char kCoverageExecutionStatusTopic[] =
  "/savo_mapping/coverage_execution/status";

inline constexpr char kCoverageExecutionFeedbackTopic[] =
  "/savo_mapping/coverage_execution/feedback";

inline constexpr char kCoverageExecutionApproveService[] =
  "/savo_mapping/coverage_execution/approve";

inline constexpr char kCoverageExecutionCancelService[] =
  "/savo_mapping/coverage_execution/cancel";

inline constexpr char kCoverageExecutionResetService[] =
  "/savo_mapping/coverage_execution/reset";

struct CoverageExecutionHandoffPolicy
{
  double server_wait_timeout_sec{3.0};
  double goal_response_timeout_sec{3.0};
  double cancel_timeout_sec{5.0};
  double execution_timeout_sec{0.0};
  std::uint32_t maximum_waypoints{10000U};
};

std::string validate_coverage_execution_handoff_policy(
  const CoverageExecutionHandoffPolicy & policy);

std::string make_coverage_mission_id(
  const std::string & prefix,
  std::uint64_t candidate_generation,
  std::int64_t event_time_ns);

}  // namespace savo_mapping::coverage
