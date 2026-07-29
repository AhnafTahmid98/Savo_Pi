// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/coverage_action_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "builtin_interfaces/msg/duration.hpp"

namespace
{

using Action =
  savo_msgs::action::ExecuteCoveragePath;

bool PolicyValid(
  const savo_nav::CoverageActionAdapterPolicy & policy)
{
  return
    std::isfinite(
      policy.default_execution_timeout_seconds) &&
    policy.default_execution_timeout_seconds > 0.0 &&
    std::isfinite(
      policy.maximum_execution_timeout_seconds) &&
    policy.maximum_execution_timeout_seconds >=
    policy.default_execution_timeout_seconds;
}

bool DurationToSeconds(
  const builtin_interfaces::msg::Duration & duration,
  double & seconds)
{
  if (
    duration.sec < 0 ||
    duration.nanosec >= 1000000000U)
  {
    return false;
  }

  seconds =
    static_cast<double>(duration.sec) +
    static_cast<double>(duration.nanosec) * 1.0e-9;

  return std::isfinite(seconds);
}

builtin_interfaces::msg::Duration SecondsToDuration(
  const double input_seconds)
{
  builtin_interfaces::msg::Duration duration;

  if (
    !std::isfinite(input_seconds) ||
    input_seconds <= 0.0)
  {
    return duration;
  }

  const double maximum_seconds =
    static_cast<double>(
    std::numeric_limits<std::int32_t>::max());

  const double seconds =
    std::min(input_seconds, maximum_seconds);

  duration.sec =
    static_cast<std::int32_t>(std::floor(seconds));

  double fractional =
    seconds - static_cast<double>(duration.sec);

  std::uint64_t nanoseconds =
    static_cast<std::uint64_t>(
    std::llround(fractional * 1.0e9));

  if (nanoseconds >= 1000000000ULL) {
    if (
      duration.sec <
      std::numeric_limits<std::int32_t>::max())
    {
      ++duration.sec;
      nanoseconds = 0;
    } else {
      nanoseconds = 999999999ULL;
    }
  }

  duration.nanosec =
    static_cast<std::uint32_t>(nanoseconds);

  return duration;
}

std::uint8_t FeedbackState(
  const savo_nav::CoverageExecutionState state)
{
  switch (state) {
    case savo_nav::CoverageExecutionState::kIdle:
      return Action::Feedback::STATE_WAITING_FOR_ADMISSION;

    case savo_nav::CoverageExecutionState::kWaitingForBackend:
      return Action::Feedback::STATE_WAITING_FOR_BACKEND;

    case savo_nav::CoverageExecutionState::kExecuting:
      return Action::Feedback::STATE_EXECUTING;

    case savo_nav::CoverageExecutionState::kCanceling:
    case savo_nav::CoverageExecutionState::kTimedOut:
    case savo_nav::CoverageExecutionState::kFeedbackStale:
      return Action::Feedback::STATE_CANCELING;

    case savo_nav::CoverageExecutionState::kSucceeded:
    case savo_nav::CoverageExecutionState::kRejected:
    case savo_nav::CoverageExecutionState::kFailed:
    case savo_nav::CoverageExecutionState::kCanceled:
      return Action::Feedback::STATE_EXECUTING;
  }

  return Action::Feedback::STATE_WAITING_FOR_ADMISSION;
}

std::uint8_t ResultCode(
  const savo_nav::CoverageTerminalCode code)
{
  switch (code) {
    case savo_nav::CoverageTerminalCode::kSucceeded:
      return Action::Result::RESULT_SUCCEEDED;
    case savo_nav::CoverageTerminalCode::kInvalidRequest:
      return Action::Result::RESULT_INVALID_REQUEST;
    case savo_nav::CoverageTerminalCode::kNotReady:
      return Action::Result::RESULT_NOT_READY;
    case savo_nav::CoverageTerminalCode::kBusy:
      return Action::Result::RESULT_BUSY;
    case savo_nav::CoverageTerminalCode::kBackendUnavailable:
      return Action::Result::RESULT_BACKEND_UNAVAILABLE;
    case savo_nav::CoverageTerminalCode::kBackendRejected:
      return Action::Result::RESULT_BACKEND_REJECTED;
    case savo_nav::CoverageTerminalCode::kAborted:
      return Action::Result::RESULT_ABORTED;
    case savo_nav::CoverageTerminalCode::kCanceled:
      return Action::Result::RESULT_CANCELED;
    case savo_nav::CoverageTerminalCode::kTimedOut:
      return Action::Result::RESULT_TIMED_OUT;
    case savo_nav::CoverageTerminalCode::kFeedbackStale:
      return Action::Result::RESULT_FEEDBACK_STALE;
    case savo_nav::CoverageTerminalCode::kInternalError:
    case savo_nav::CoverageTerminalCode::kNone:
      return Action::Result::RESULT_INTERNAL_ERROR;
  }

  return Action::Result::RESULT_INTERNAL_ERROR;
}

}  // namespace

namespace savo_nav
{

CoverageAdaptedGoal CoverageActionAdapter::AdaptGoal(
  const ExecuteCoveragePath::Goal & goal,
  const MapContext & map_context,
  const NavigationReadinessResult & readiness,
  const std::uint64_t sequence,
  const CoverageActionAdapterPolicy & policy)
{
  CoverageAdaptedGoal adapted;

  adapted.context.goal_id = goal.mission_id;
  adapted.context.source = GoalSource::kCoverage;
  adapted.context.target_frame = goal.path.header.frame_id;
  adapted.context.map_id = map_context.map_id;
  adapted.context.sequence = sequence;

  adapted.validation_request.contract_version =
    goal.contract_version;

  adapted.validation_request.mission_id =
    goal.mission_id;

  adapted.validation_request.path_frame =
    goal.path.header.frame_id;

  adapted.validation_request.map_context =
    map_context;

  adapted.validation_request.readiness =
    readiness;

  adapted.validation_request.points.reserve(
    goal.path.poses.size());

  for (const auto & pose_stamped : goal.path.poses) {
    CoveragePathPoint point;

    point.x = pose_stamped.pose.position.x;
    point.y = pose_stamped.pose.position.y;
    point.z = pose_stamped.pose.position.z;

    point.orientation_x =
      pose_stamped.pose.orientation.x;

    point.orientation_y =
      pose_stamped.pose.orientation.y;

    point.orientation_z =
      pose_stamped.pose.orientation.z;

    point.orientation_w =
      pose_stamped.pose.orientation.w;

    point.frame_id =
      pose_stamped.header.frame_id;

    adapted.validation_request.points.push_back(
      std::move(point));
  }

  if (!PolicyValid(policy)) {
    adapted.validation.validation = {
      ValidationCode::kInvalidCombination,
      "coverage_action_adapter_policy_is_invalid"
    };

    return adapted;
  }

  double requested_timeout_seconds = 0.0;

  if (
    !DurationToSeconds(
      goal.execution_timeout,
      requested_timeout_seconds))
  {
    adapted.validation.validation = {
      ValidationCode::kInvalidCombination,
      "coverage_execution_timeout_duration_is_invalid"
    };

    return adapted;
  }

  adapted.requested_execution_timeout_seconds =
    requested_timeout_seconds;

  adapted.resolved_execution_timeout_seconds =
    requested_timeout_seconds == 0.0 ?
    policy.default_execution_timeout_seconds :
    std::min(
    requested_timeout_seconds,
    policy.maximum_execution_timeout_seconds);

  adapted.validation_request.execution_timeout_seconds =
    adapted.resolved_execution_timeout_seconds;

  auto validation_policy = policy.validation_policy;

  validation_policy.maximum_execution_timeout_seconds =
    policy.maximum_execution_timeout_seconds;

  adapted.validation =
    CoveragePathValidator::Validate(
    adapted.validation_request,
    validation_policy);

  return adapted;
}

CoverageActionAdapter::ExecuteCoveragePath::Feedback
CoverageActionAdapter::MakeFeedback(
  const CoverageExecutionSnapshot & execution,
  const CoverageProgressSnapshot & progress)
{
  ExecuteCoveragePath::Feedback feedback;

  feedback.state = FeedbackState(execution.state);

  feedback.state_text = std::string(
    CoverageExecutionModel::ToString(
      execution.state));

  feedback.reason = execution.reason;

  feedback.current_waypoint =
    progress.current_waypoint;

  feedback.completed_waypoints =
    progress.completed_waypoints;

  feedback.total_waypoints =
    progress.total_waypoints;

  feedback.completion_ratio =
    progress.completion_ratio;

  feedback.remaining_distance_m =
    progress.remaining_distance_m;

  feedback.elapsed_time =
    SecondsToDuration(progress.elapsed_seconds);

  feedback.estimated_time_remaining =
    SecondsToDuration(
    progress.estimated_remaining_seconds);

  return feedback;
}

CoverageActionAdapter::ExecuteCoveragePath::Result
CoverageActionAdapter::MakeResult(
  const CoverageExecutionSnapshot & execution,
  const CoverageProgressSnapshot & progress)
{
  ExecuteCoveragePath::Result result;

  result.success =
    execution.terminal_code ==
    CoverageTerminalCode::kSucceeded;

  result.result_code =
    ResultCode(execution.terminal_code);

  result.terminal_state = std::string(
    CoverageExecutionModel::ToString(
      execution.state));

  result.reason = execution.reason;

  result.completed_waypoints =
    progress.completed_waypoints;

  result.total_waypoints =
    progress.total_waypoints;

  result.completion_ratio =
    progress.completion_ratio;

  result.remaining_distance_m =
    progress.remaining_distance_m;

  return result;
}

}  // namespace savo_nav
