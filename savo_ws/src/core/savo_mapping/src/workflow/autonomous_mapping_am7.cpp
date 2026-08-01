#include "savo_mapping/autonomous_mapping_am7.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace savo_mapping::autonomous
{
namespace
{

constexpr double kTwoPi = 6.283185307179586476925286766559;

template<typename T>
T value_or(const nlohmann::json & value, const char * key, T fallback)
{
  const auto iterator = value.find(key);
  if (iterator == value.end() || iterator->is_null()) {
    return fallback;
  }
  try {
    return iterator->get<T>();
  } catch (const nlohmann::json::exception &) {
    return fallback;
  }
}

bool finite_nonnegative(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool normalize_planar_quaternion(
  const PlanarPose & pose,
  double & yaw)
{
  if (!std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
    !std::isfinite(pose.quaternion_z) ||
    !std::isfinite(pose.quaternion_w))
  {
    return false;
  }
  const double norm = std::hypot(pose.quaternion_z, pose.quaternion_w);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  yaw = 2.0 * std::atan2(
    pose.quaternion_z / norm,
    pose.quaternion_w / norm);
  return std::isfinite(yaw);
}

}  // namespace

std::string_view to_string(const ReturnActionLifecycle state)
{
  switch (state) {
    case ReturnActionLifecycle::Idle:
      return "idle";
    case ReturnActionLifecycle::WaitingForServer:
      return "waiting_for_server";
    case ReturnActionLifecycle::GoalRequestPending:
      return "goal_request_pending";
    case ReturnActionLifecycle::AcceptedActive:
      return "accepted_active";
    case ReturnActionLifecycle::CancelPending:
      return "cancel_pending";
    case ReturnActionLifecycle::Terminal:
      return "terminal";
    case ReturnActionLifecycle::VerifyingProximity:
      return "verifying_proximity";
  }
  return "unknown";
}

bool return_goal_may_be_executing(const ReturnActionLifecycle state)
{
  return
    state == ReturnActionLifecycle::GoalRequestPending ||
    state == ReturnActionLifecycle::AcceptedActive ||
    state == ReturnActionLifecycle::CancelPending;
}

CoveragePlannerObservation parse_coverage_planner_status(
  const std::string & json_text)
{
  CoveragePlannerObservation output;
  try {
    const auto status = nlohmann::json::parse(json_text);
    if (!status.is_object()) {
      return output;
    }
    output.parsed = true;
    output.state = value_or(status, "state", output.state);
    output.reason = value_or(status, "reason", output.reason);
    output.map_valid = value_or(status, "map_valid", false);
    output.map_fresh = value_or(status, "map_fresh", false);
    output.request_generation = value_or<std::uint64_t>(
      status, "request_generation", 0U);
    output.reset_generation = value_or<std::uint64_t>(
      status, "reset_generation", 0U);
    output.plan_generation = value_or<std::uint64_t>(
      status, "plan_sequence", 0U);
    output.map_generation = value_or<std::uint64_t>(
      status, "map_sequence", 0U);
    output.waypoint_count = value_or<std::uint32_t>(
      status, "waypoint_count", 0U);
    output.explicit_noop = value_or(status, "explicit_noop", false);
    output.planning_started =
      output.state == "planning_requested" ||
      output.state == "waiting_for_pose" ||
      output.state == "planning" ||
      output.state == "plan_ready" ||
      output.state == "plan_failed";
    output.planning_complete =
      output.state == "plan_ready" || output.state == "plan_failed";
    output.plan_valid =
      output.state == "plan_ready" && output.map_valid && output.map_fresh &&
      (output.waypoint_count > 0U || output.explicit_noop);
  } catch (const nlohmann::json::exception &) {
    output.reason = "coverage_status_json_invalid";
  }
  return output;
}

CoverageOperationObservation parse_coverage_operation_status(
  const std::string & json_text)
{
  CoverageOperationObservation output;
  try {
    const auto status = nlohmann::json::parse(json_text);
    if (!status.is_object()) {
      return output;
    }
    output.parsed = true;
    output.supervisor_authorized = value_or(
      status, "supervisor_authorized", false);
    output.candidate_valid = value_or(status, "candidate_valid", false);
    output.approval_pending = value_or(status, "approval_pending", false);
    output.feedback_received = value_or(
      status, "feedback_received", false);
    output.feedback_sequence = value_or<std::uint64_t>(
      status, "feedback_sequence", 0U);
    output.feedback_age_s = value_or(status, "feedback_age_s", -1.0);
    output.candidate_generation = value_or<std::uint64_t>(
      status, "candidate_generation", 0U);
    output.state = value_or(status, "state", output.state);
    output.mission_id = value_or(status, "mission_id", std::string{});
    output.terminal_state = value_or(
      status, "terminal_state", std::string{});
    output.reason = value_or(status, "result_reason", output.reason);

    try {
      const auto feedback_text = value_or(
        status, "latest_feedback", std::string{});
      if (!feedback_text.empty()) {
        const auto feedback = nlohmann::json::parse(feedback_text);
        if (feedback.is_object() &&
          value_or(feedback, "mission_id", std::string{}) == output.mission_id)
        {
          output.current_waypoint = value_or<std::uint32_t>(
            feedback, "current_waypoint", 0U);
          output.completed_waypoints = value_or<std::uint32_t>(
            feedback, "completed_waypoints", 0U);
          output.total_waypoints = value_or<std::uint32_t>(
            feedback, "total_waypoints", 0U);
          output.completion_ratio = value_or(
            feedback, "completion_ratio", 0.0);
          output.remaining_distance_m = value_or(
            feedback, "remaining_distance_m", 0.0);
        }
      }
    } catch (const nlohmann::json::exception &) {
      // A malformed nested feedback sample must not discard the valid,
      // independently correlated operation state around it.
    }
  } catch (const nlohmann::json::exception &) {
    output.reason = "coverage_operation_status_json_invalid";
  }
  return output;
}

CoveragePlanCorrelationResult evaluate_coverage_plan_correlation(
  const CoveragePlannerObservation & observation,
  const CoveragePlanCorrelation & expected)
{
  CoveragePlanCorrelationResult output;
  output.current_request = expected.expected_request_generation > 0U &&
    observation.request_generation == expected.expected_request_generation;
  output.current_reset = expected.expected_reset_generation > 0U &&
    observation.reset_generation >= expected.expected_reset_generation;
  output.current_plan =
    observation.plan_generation > expected.plan_generation_floor;
  output.current_map = !expected.require_fresh_map_generation ||
    (observation.map_generation > 0U &&
    observation.map_generation >= expected.map_generation_floor);

  if (!output.current_reset) {
    output.reason = "coverage_plan_reset_generation_stale";
  } else if (!output.current_request) {
    output.reason = "coverage_plan_request_generation_stale";
  } else if (!output.current_plan) {
    output.reason = "coverage_plan_generation_stale";
  } else if (!output.current_map) {
    output.reason = "coverage_plan_map_generation_stale";
  } else if (!observation.plan_valid) {
    output.reason = observation.reason.empty() ?
      "coverage_plan_invalid" : observation.reason;
  } else {
    output.accepted = true;
    output.reason = observation.reason;
  }
  return output;
}

ProximityResult evaluate_planar_proximity(
  const PlanarPose & target,
  const PlanarPose & actual,
  const double position_tolerance_m,
  const bool require_yaw_tolerance,
  const double yaw_tolerance_rad)
{
  ProximityResult output;
  if (!finite_nonnegative(position_tolerance_m) ||
    !finite_nonnegative(yaw_tolerance_rad))
  {
    output.reason = "pose_proximity_tolerance_invalid";
    return output;
  }

  double target_yaw = 0.0;
  double actual_yaw = 0.0;
  if (!normalize_planar_quaternion(target, target_yaw) ||
    !normalize_planar_quaternion(actual, actual_yaw))
  {
    output.reason = "pose_proximity_pose_invalid";
    return output;
  }

  output.distance_m = std::hypot(actual.x_m - target.x_m, actual.y_m - target.y_m);
  output.yaw_error_rad = std::abs(
    std::remainder(actual_yaw - target_yaw, kTwoPi));
  output.valid = std::isfinite(output.distance_m) &&
    std::isfinite(output.yaw_error_rad);
  output.within_tolerance = output.valid &&
    output.distance_m <= position_tolerance_m &&
    (!require_yaw_tolerance || output.yaw_error_rad <= yaw_tolerance_rad);
  output.reason = output.within_tolerance ?
    "return_to_start_within_tolerance" : "return_to_start_outside_tolerance";
  return output;
}

}  // namespace savo_mapping::autonomous
