#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace savo_mapping::autonomous
{

enum class ReturnActionLifecycle : std::uint8_t
{
  Idle = 0,
  WaitingForServer = 1,
  GoalRequestPending = 2,
  AcceptedActive = 3,
  CancelPending = 4,
  Terminal = 5,
  VerifyingProximity = 6,
};

std::string_view to_string(ReturnActionLifecycle state);

bool return_goal_may_be_executing(ReturnActionLifecycle state);

struct CoveragePlannerObservation
{
  bool parsed{false};
  bool planning_started{false};
  bool planning_complete{false};
  bool plan_valid{false};
  bool explicit_noop{false};
  bool map_valid{false};
  bool map_fresh{false};
  std::uint64_t request_generation{0};
  std::uint64_t reset_generation{0};
  std::uint64_t plan_generation{0};
  std::uint64_t map_generation{0};
  std::uint32_t waypoint_count{0};
  std::string state{"unavailable"};
  std::string reason{"coverage_status_unavailable"};
};

struct CoverageOperationObservation
{
  bool parsed{false};
  bool supervisor_authorized{false};
  bool candidate_valid{false};
  bool approval_pending{false};
  bool feedback_received{false};
  std::uint64_t feedback_sequence{0};
  double feedback_age_s{-1.0};
  std::uint64_t candidate_generation{0};
  std::string state{"unavailable"};
  std::string mission_id;
  std::string terminal_state;
  std::string reason{"coverage_operation_status_unavailable"};
  std::uint32_t current_waypoint{0};
  std::uint32_t completed_waypoints{0};
  std::uint32_t total_waypoints{0};
  double completion_ratio{0.0};
  double remaining_distance_m{0.0};
};

struct PlanarPose
{
  double x_m{0.0};
  double y_m{0.0};
  double quaternion_z{0.0};
  double quaternion_w{1.0};
};

struct ProximityResult
{
  bool valid{false};
  bool within_tolerance{false};
  double distance_m{0.0};
  double yaw_error_rad{0.0};
  std::string reason{"pose_proximity_not_evaluated"};
};

CoveragePlannerObservation parse_coverage_planner_status(
  const std::string & json_text);

CoverageOperationObservation parse_coverage_operation_status(
  const std::string & json_text);

ProximityResult evaluate_planar_proximity(
  const PlanarPose & target,
  const PlanarPose & actual,
  double position_tolerance_m,
  bool require_yaw_tolerance,
  double yaw_tolerance_rad);

}  // namespace savo_mapping::autonomous
