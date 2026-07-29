#include "savo_mapping/coverage_mission.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace savo_mapping::coverage
{

std::string to_string(
  const CoverageMissionState state)
{
  switch (state) {
    case CoverageMissionState::kIdle:
      return "idle";
    case CoverageMissionState::kPlanReady:
      return "plan_ready";
    case CoverageMissionState::kAwaitingDispatch:
      return "awaiting_dispatch";
    case CoverageMissionState::kExecuting:
      return "executing";
    case CoverageMissionState::kCanceling:
      return "canceling";
    case CoverageMissionState::kSucceeded:
      return "succeeded";
    case CoverageMissionState::kCanceled:
      return "canceled";
    case CoverageMissionState::kFailed:
      return "failed";
    case CoverageMissionState::kRejected:
      return "rejected";
    case CoverageMissionState::kTimedOut:
      return "timed_out";
  }

  return "unknown";
}

bool is_active(
  const CoverageMissionState state)
{
  switch (state) {
    case CoverageMissionState::kAwaitingDispatch:
    case CoverageMissionState::kExecuting:
    case CoverageMissionState::kCanceling:
      return true;

    default:
      return false;
  }
}

bool is_terminal(
  const CoverageMissionState state)
{
  switch (state) {
    case CoverageMissionState::kSucceeded:
    case CoverageMissionState::kCanceled:
    case CoverageMissionState::kFailed:
    case CoverageMissionState::kRejected:
    case CoverageMissionState::kTimedOut:
      return true;

    default:
      return false;
  }
}

bool transition_allowed(
  const CoverageMissionState from,
  const CoverageMissionState to)
{
  switch (from) {
    case CoverageMissionState::kIdle:
      return to == CoverageMissionState::kPlanReady;

    case CoverageMissionState::kPlanReady:
      return
        to == CoverageMissionState::kAwaitingDispatch ||
        to == CoverageMissionState::kCanceled;

    case CoverageMissionState::kAwaitingDispatch:
      return
        to == CoverageMissionState::kExecuting ||
        to == CoverageMissionState::kRejected ||
        to == CoverageMissionState::kCanceled ||
        to == CoverageMissionState::kFailed ||
        to == CoverageMissionState::kTimedOut;

    case CoverageMissionState::kExecuting:
      return
        to == CoverageMissionState::kCanceling ||
        to == CoverageMissionState::kSucceeded ||
        to == CoverageMissionState::kCanceled ||
        to == CoverageMissionState::kFailed ||
        to == CoverageMissionState::kTimedOut;

    case CoverageMissionState::kCanceling:
      return
        to == CoverageMissionState::kSucceeded ||
        to == CoverageMissionState::kCanceled ||
        to == CoverageMissionState::kFailed ||
        to == CoverageMissionState::kTimedOut;

    case CoverageMissionState::kSucceeded:
    case CoverageMissionState::kCanceled:
    case CoverageMissionState::kFailed:
    case CoverageMissionState::kRejected:
    case CoverageMissionState::kTimedOut:
      return false;
  }

  return false;
}

}  // namespace savo_mapping::coverage

namespace savo_mapping::coverage
{

CoverageMissionState
CoverageMission::state() const noexcept
{
  return state_;
}

const CoverageMissionPlan &
CoverageMission::plan() const noexcept
{
  return plan_;
}

const CoverageMissionProgress &
CoverageMission::progress() const noexcept
{
  return progress_;
}

const std::string &
CoverageMission::reason() const noexcept
{
  return reason_;
}

std::int64_t
CoverageMission::started_at_ns() const noexcept
{
  return started_at_ns_;
}

std::uint64_t
CoverageMission::sequence() const noexcept
{
  return sequence_;
}

bool CoverageMission::event_time_valid(
  const std::int64_t event_time_ns) const noexcept
{
  if (event_time_ns < 0) {
    return false;
  }

  if (progress_.latest_event_ns < 0) {
    return true;
  }

  return event_time_ns >
         progress_.latest_event_ns;
}

CoverageMissionTransition
CoverageMission::load_plan(
  const CoverageMissionPlan & plan)
{
  if (state_ != CoverageMissionState::kIdle) {
    return rejected_event(
      "mission_must_be_idle_before_plan_load");
  }

  if (plan.mission_id.empty()) {
    return rejected_event(
      "mission_id_empty");
  }

  if (plan.total_waypoints == 0U) {
    return rejected_event(
      "plan_has_no_waypoints");
  }

  if (!std::isfinite(plan.total_distance_m) ||
    plan.total_distance_m < 0.0)
  {
    return rejected_event(
      "plan_distance_invalid");
  }

  if (plan.created_at_ns < 0) {
    return rejected_event(
      "plan_timestamp_invalid");
  }

  const CoverageMissionState previous =
    state_;

  plan_ = plan;

  progress_ = CoverageMissionProgress{};
  progress_.remaining_distance_m =
    plan_.total_distance_m;
  progress_.latest_event_ns =
    plan_.created_at_ns;

  state_ = CoverageMissionState::kPlanReady;
  reason_ = "plan_loaded";
  started_at_ns_ = -1;

  ++sequence_;

  return CoverageMissionTransition{
    true,
    previous,
    state_,
    plan_.mission_id,
    reason_,
    sequence_};
}

CoverageMissionTransition
CoverageMission::rejected_event(
  const std::string & reason) const
{
  return CoverageMissionTransition{
    false,
    state_,
    state_,
    plan_.mission_id,
    reason,
    sequence_};
}

}  // namespace savo_mapping::coverage

namespace savo_mapping::coverage
{

CoverageMissionTransition
CoverageMission::transition(
  const CoverageMissionState next,
  const std::string & reason,
  const std::int64_t event_time_ns)
{
  if (!event_time_valid(event_time_ns)) {
    return rejected_event(
      "event_time_not_newer");
  }

  if (!transition_allowed(state_, next)) {
    return rejected_event(
      "invalid_transition:" +
      to_string(state_) +
      "_to_" +
      to_string(next));
  }

  const CoverageMissionState previous =
    state_;

  state_ = next;
  reason_ = reason.empty() ?
    "state_changed" :
    reason;

  progress_.latest_event_ns =
    event_time_ns;

  if (state_ == CoverageMissionState::kExecuting &&
    started_at_ns_ < 0)
  {
    started_at_ns_ = event_time_ns;
  }

  ++sequence_;

  return CoverageMissionTransition{
    true,
    previous,
    state_,
    plan_.mission_id,
    reason_,
    sequence_};
}

CoverageMissionTransition
CoverageMission::accepted_event(
  const std::string & reason,
  const std::int64_t event_time_ns)
{
  if (!event_time_valid(event_time_ns)) {
    return rejected_event(
      "event_time_not_newer");
  }

  reason_ = reason.empty() ?
    "event_accepted" :
    reason;

  progress_.latest_event_ns =
    event_time_ns;

  ++sequence_;

  return CoverageMissionTransition{
    true,
    state_,
    state_,
    plan_.mission_id,
    reason_,
    sequence_};
}

CoverageMissionTransition
CoverageMission::request_dispatch(
  const std::int64_t event_time_ns)
{
  if (state_ !=
    CoverageMissionState::kPlanReady)
  {
    return rejected_event(
      "plan_not_ready_for_dispatch");
  }

  return transition(
    CoverageMissionState::kAwaitingDispatch,
    "dispatch_requested",
    event_time_ns);
}

CoverageMissionTransition
CoverageMission::mark_accepted(
  const std::int64_t event_time_ns)
{
  if (state_ !=
    CoverageMissionState::kAwaitingDispatch)
  {
    return rejected_event(
      "mission_not_awaiting_acceptance");
  }

  return transition(
    CoverageMissionState::kExecuting,
    "execution_accepted",
    event_time_ns);
}

CoverageMissionTransition
CoverageMission::update_progress(
  const std::string & mission_id,
  const std::uint32_t current_waypoint,
  const std::uint32_t completed_waypoints,
  const double completed_distance_m,
  const std::int64_t event_time_ns)
{
  if (state_ != CoverageMissionState::kExecuting &&
    state_ != CoverageMissionState::kCanceling)
  {
    return rejected_event(
      "mission_not_active_for_progress");
  }

  if (mission_id != plan_.mission_id) {
    return rejected_event(
      "mission_id_mismatch");
  }

  if (!event_time_valid(event_time_ns)) {
    return rejected_event(
      "event_time_not_newer");
  }

  if (completed_waypoints >
    plan_.total_waypoints)
  {
    return rejected_event(
      "completed_waypoints_out_of_range");
  }

  if (current_waypoint != kNoWaypoint &&
    current_waypoint >= plan_.total_waypoints)
  {
    return rejected_event(
      "current_waypoint_out_of_range");
  }

  if (current_waypoint != kNoWaypoint &&
    current_waypoint < completed_waypoints &&
    completed_waypoints < plan_.total_waypoints)
  {
    return rejected_event(
      "current_waypoint_before_completed");
  }

  if (completed_waypoints <
    progress_.completed_waypoints)
  {
    return rejected_event(
      "completed_waypoints_regressed");
  }

  if (!std::isfinite(completed_distance_m) ||
    completed_distance_m < 0.0 ||
    completed_distance_m >
    plan_.total_distance_m + 1.0e-9)
  {
    return rejected_event(
      "completed_distance_invalid");
  }

  if (completed_distance_m + 1.0e-9 <
    progress_.completed_distance_m)
  {
    return rejected_event(
      "completed_distance_regressed");
  }

  progress_.current_waypoint =
    current_waypoint;

  progress_.completed_waypoints =
    completed_waypoints;

  progress_.completed_distance_m =
    std::min(
    completed_distance_m,
    plan_.total_distance_m);

  progress_.remaining_distance_m =
    std::max(
    0.0,
    plan_.total_distance_m -
    progress_.completed_distance_m);

  const double waypoint_ratio =
    static_cast<double>(
    completed_waypoints) /
    static_cast<double>(
    plan_.total_waypoints);

  const double ratio =
    plan_.total_distance_m > 1.0e-9 ?
    progress_.completed_distance_m /
    plan_.total_distance_m :
    waypoint_ratio;

  progress_.completion_ratio =
    std::clamp(ratio, 0.0, 1.0);

  return accepted_event(
    "progress_updated",
    event_time_ns);
}

CoverageMissionTransition
CoverageMission::request_cancel(
  const std::int64_t event_time_ns,
  const std::string & reason)
{
  const std::string resolved_reason =
    reason.empty() ?
    "cancel_requested" :
    reason;

  if (state_ == CoverageMissionState::kPlanReady ||
    state_ == CoverageMissionState::kAwaitingDispatch)
  {
    return transition(
      CoverageMissionState::kCanceled,
      resolved_reason,
      event_time_ns);
  }

  if (state_ == CoverageMissionState::kExecuting) {
    return transition(
      CoverageMissionState::kCanceling,
      resolved_reason,
      event_time_ns);
  }

  return rejected_event(
    "mission_not_cancelable");
}

CoverageMissionTransition
CoverageMission::mark_succeeded(
  const std::int64_t event_time_ns)
{
  auto result = transition(
    CoverageMissionState::kSucceeded,
    "mission_succeeded",
    event_time_ns);

  if (result.accepted) {
    progress_.current_waypoint =
      kNoWaypoint;
    progress_.completed_waypoints =
      plan_.total_waypoints;
    progress_.completed_distance_m =
      plan_.total_distance_m;
    progress_.remaining_distance_m = 0.0;
    progress_.completion_ratio = 1.0;
  }

  return result;
}

CoverageMissionTransition
CoverageMission::mark_canceled(
  const std::int64_t event_time_ns,
  const std::string & reason)
{
  return transition(
    CoverageMissionState::kCanceled,
    reason.empty() ?
    "mission_canceled" :
    reason,
    event_time_ns);
}

CoverageMissionTransition
CoverageMission::mark_failed(
  const std::int64_t event_time_ns,
  const std::string & reason)
{
  return transition(
    CoverageMissionState::kFailed,
    reason.empty() ?
    "mission_failed" :
    reason,
    event_time_ns);
}

CoverageMissionTransition
CoverageMission::mark_rejected(
  const std::int64_t event_time_ns,
  const std::string & reason)
{
  return transition(
    CoverageMissionState::kRejected,
    reason.empty() ?
    "mission_rejected" :
    reason,
    event_time_ns);
}

CoverageMissionTransition
CoverageMission::mark_timed_out(
  const std::int64_t event_time_ns,
  const std::string & reason)
{
  return transition(
    CoverageMissionState::kTimedOut,
    reason.empty() ?
    "mission_timed_out" :
    reason,
    event_time_ns);
}

CoverageMissionTransition
CoverageMission::reset()
{
  if (state_ == CoverageMissionState::kIdle) {
    return CoverageMissionTransition{
      true,
      state_,
      state_,
      plan_.mission_id,
      "already_idle",
      sequence_};
  }

  if (!is_terminal(state_)) {
    return rejected_event(
      "cannot_reset_nonterminal_mission");
  }

  const CoverageMissionState previous =
    state_;

  const std::string previous_mission_id =
    plan_.mission_id;

  ++sequence_;
  clear();

  return CoverageMissionTransition{
    true,
    previous,
    state_,
    previous_mission_id,
    reason_,
    sequence_};
}

void CoverageMission::clear() noexcept
{
  state_ = CoverageMissionState::kIdle;
  plan_ = CoverageMissionPlan{};
  progress_ = CoverageMissionProgress{};
  reason_ = "idle";
  started_at_ns_ = -1;
}

}  // namespace savo_mapping::coverage
