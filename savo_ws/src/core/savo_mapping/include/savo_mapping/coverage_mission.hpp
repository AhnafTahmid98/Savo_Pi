#pragma once

#include <cstdint>
#include <limits>
#include <string>

namespace savo_mapping::coverage
{

inline constexpr std::uint32_t kNoWaypoint =
  std::numeric_limits<std::uint32_t>::max();

enum class CoverageMissionState
{
  kIdle,
  kPlanReady,
  kAwaitingDispatch,
  kExecuting,
  kCanceling,
  kSucceeded,
  kCanceled,
  kFailed,
  kRejected,
  kTimedOut,
};

struct CoverageMissionPlan
{
  std::string mission_id;
  std::uint32_t total_waypoints{0};
  double total_distance_m{0.0};
  std::int64_t created_at_ns{0};
};

struct CoverageMissionProgress
{
  std::uint32_t current_waypoint{kNoWaypoint};
  std::uint32_t completed_waypoints{0};

  double completed_distance_m{0.0};
  double remaining_distance_m{0.0};
  double completion_ratio{0.0};

  std::int64_t latest_event_ns{-1};
};

struct CoverageMissionTransition
{
  bool accepted{false};

  CoverageMissionState previous{
    CoverageMissionState::kIdle};

  CoverageMissionState current{
    CoverageMissionState::kIdle};

  std::string mission_id;
  std::string reason;

  std::uint64_t sequence{0};
};

std::string to_string(
  CoverageMissionState state);

bool is_active(
  CoverageMissionState state);

bool is_terminal(
  CoverageMissionState state);

bool transition_allowed(
  CoverageMissionState from,
  CoverageMissionState to);

class CoverageMission
{
public:
  CoverageMission() = default;

  CoverageMissionState state() const noexcept;

  const CoverageMissionPlan & plan() const noexcept;

  const CoverageMissionProgress &
  progress() const noexcept;

  const std::string & reason() const noexcept;

  std::int64_t started_at_ns() const noexcept;

  std::uint64_t sequence() const noexcept;

  CoverageMissionTransition load_plan(
    const CoverageMissionPlan & plan);

  CoverageMissionTransition request_dispatch(
    std::int64_t event_time_ns);

  CoverageMissionTransition mark_accepted(
    std::int64_t event_time_ns);

  CoverageMissionTransition update_progress(
    const std::string & mission_id,
    std::uint32_t current_waypoint,
    std::uint32_t completed_waypoints,
    double completed_distance_m,
    std::int64_t event_time_ns);

  CoverageMissionTransition request_cancel(
    std::int64_t event_time_ns,
    const std::string & reason =
    "cancel_requested");

  CoverageMissionTransition mark_succeeded(
    std::int64_t event_time_ns);

  CoverageMissionTransition mark_canceled(
    std::int64_t event_time_ns,
    const std::string & reason =
    "mission_canceled");

  CoverageMissionTransition mark_failed(
    std::int64_t event_time_ns,
    const std::string & reason =
    "mission_failed");

  CoverageMissionTransition mark_rejected(
    std::int64_t event_time_ns,
    const std::string & reason =
    "mission_rejected");

  CoverageMissionTransition mark_timed_out(
    std::int64_t event_time_ns,
    const std::string & reason =
    "mission_timed_out");

  CoverageMissionTransition reset();

private:
  CoverageMissionTransition transition(
    CoverageMissionState next,
    const std::string & reason,
    std::int64_t event_time_ns);

  CoverageMissionTransition accepted_event(
    const std::string & reason,
    std::int64_t event_time_ns);

  CoverageMissionTransition rejected_event(
    const std::string & reason) const;

  bool event_time_valid(
    std::int64_t event_time_ns) const noexcept;

  void clear() noexcept;

  CoverageMissionState state_{
    CoverageMissionState::kIdle};

  CoverageMissionPlan plan_;
  CoverageMissionProgress progress_;

  std::string reason_{"idle"};

  std::int64_t started_at_ns_{-1};
  std::uint64_t sequence_{0};
};

}  // namespace savo_mapping::coverage
