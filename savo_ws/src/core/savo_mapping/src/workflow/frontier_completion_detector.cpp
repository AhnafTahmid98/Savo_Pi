#include "savo_mapping/frontier_completion_detector.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace savo_mapping::autonomous
{

std::string_view to_string(const FrontierExhaustionKind kind)
{
  switch (kind) {
    case FrontierExhaustionKind::None:
      return "none";
    case FrontierExhaustionKind::NoFrontiers:
      return "no_frontiers";
    case FrontierExhaustionKind::NoReachableFrontiers:
      return "no_reachable_frontiers";
    case FrontierExhaustionKind::NoSelectableFrontier:
      return "no_selectable_frontier";
  }

  return "unknown";
}

std::string validate_frontier_completion_config(
  const FrontierCompletionConfig & config)
{
  if (config.minimum_observations == 0U) {
    return "minimum_observations_must_be_positive";
  }

  if (!std::isfinite(config.minimum_stable_duration_s) ||
    config.minimum_stable_duration_s < 0.0)
  {
    return "minimum_stable_duration_s_must_be_nonnegative";
  }

  if (!std::isfinite(config.status_timeout_s) ||
    config.status_timeout_s <= 0.0)
  {
    return "status_timeout_s_must_be_positive";
  }

  if (
    !config.allow_no_frontiers &&
    !config.allow_no_reachable_frontiers &&
    !config.allow_no_selectable_frontier)
  {
    return "at_least_one_exhaustion_kind_must_be_allowed";
  }

  return {};
}

FrontierCompletionDetector::FrontierCompletionDetector(
  FrontierCompletionConfig config)
: config_(std::move(config))
{
  const std::string error = validate_frontier_completion_config(config_);
  if (!error.empty()) {
    throw std::invalid_argument(error);
  }
}

FrontierCompletionSnapshot FrontierCompletionDetector::observe(
  const FrontierPlanObservation & observation,
  const double now_s)
{
  if (!std::isfinite(now_s) || now_s < 0.0) {
    throw std::invalid_argument("completion_detector_now_must_be_nonnegative");
  }

  if (!observation.received) {
    return tick(now_s);
  }

  if (!std::isfinite(observation.received_at_s) ||
    observation.received_at_s < 0.0)
  {
    throw std::invalid_argument(
            "frontier_observation_time_must_be_nonnegative");
  }

  snapshot_.status_received = true;
  last_received_at_s_ = observation.received_at_s;
  snapshot_.status_fresh =
    now_s - last_received_at_s_ <= config_.status_timeout_s;

  snapshot_.map_generation = observation.map_generation;
  snapshot_.planned_map_generation = observation.planned_map_generation;
  snapshot_.detected_frontiers = observation.detected_frontiers;
  snapshot_.reachable_frontiers = observation.reachable_frontiers;
  snapshot_.planning_status = observation.planning_status;

  if (observation.plan_sequence == 0U) {
    if (!snapshot_.confirmed) {
      reset_candidate("frontier_plan_not_available");
    }
    return snapshot_;
  }

  if (observation.plan_sequence < snapshot_.plan_sequence) {
    reset_candidate("frontier_plan_sequence_restarted");
  }

  if (observation.plan_sequence == snapshot_.plan_sequence) {
    update_freshness(now_s);
    return snapshot_;
  }

  snapshot_.plan_sequence = observation.plan_sequence;

  const FrontierExhaustionKind kind = classify(observation);
  if (
    kind == FrontierExhaustionKind::None ||
    !allowed(kind) ||
    !observation.runtime_enabled ||
    observation.goal_pending ||
    observation.handoff_active)
  {
    reset_candidate(
      kind == FrontierExhaustionKind::None ?
      "frontier_available_or_plan_not_exhausted" :
      "frontier_exhaustion_not_admissible");
    snapshot_.plan_sequence = observation.plan_sequence;
    snapshot_.map_generation = observation.map_generation;
    snapshot_.planned_map_generation = observation.planned_map_generation;
    snapshot_.detected_frontiers = observation.detected_frontiers;
    snapshot_.reachable_frontiers = observation.reachable_frontiers;
    snapshot_.planning_status = observation.planning_status;
    return snapshot_;
  }

  const bool same_candidate =
    snapshot_.candidate &&
    snapshot_.exhaustion_kind == kind &&
    candidate_map_generation_ == observation.map_generation &&
    candidate_planned_map_generation_ ==
    observation.planned_map_generation;

  if (!same_candidate) {
    snapshot_.candidate = true;
    snapshot_.confirmed = false;
    snapshot_.exhaustion_kind = kind;
    snapshot_.observations = 1U;
    snapshot_.stable_duration_s = 0.0;
    candidate_started_at_s_ = now_s;
    candidate_map_generation_ = observation.map_generation;
    candidate_planned_map_generation_ =
      observation.planned_map_generation;
  } else {
    ++snapshot_.observations;
    snapshot_.stable_duration_s = now_s - candidate_started_at_s_;
  }

  snapshot_.reason =
    std::string{"frontier_exhaustion_observed:"} +
  std::string{to_string(kind)};

  if (
    snapshot_.observations >= config_.minimum_observations &&
    snapshot_.stable_duration_s >= config_.minimum_stable_duration_s)
  {
    snapshot_.confirmed = true;
    snapshot_.reason =
      std::string{"frontier_exhaustion_confirmed:"} +
    std::string{to_string(kind)};
  }

  return snapshot_;
}

FrontierCompletionSnapshot FrontierCompletionDetector::tick(
  const double now_s)
{
  if (!std::isfinite(now_s) || now_s < 0.0) {
    throw std::invalid_argument("completion_detector_now_must_be_nonnegative");
  }

  update_freshness(now_s);
  return snapshot_;
}

void FrontierCompletionDetector::reset(std::string reason)
{
  snapshot_ = FrontierCompletionSnapshot{};
  snapshot_.reason = std::move(reason);
  candidate_started_at_s_ = 0.0;
  last_received_at_s_ = 0.0;
  candidate_map_generation_ = 0U;
  candidate_planned_map_generation_ = 0U;
}

const FrontierCompletionSnapshot &
FrontierCompletionDetector::snapshot() const noexcept
{
  return snapshot_;
}

FrontierExhaustionKind FrontierCompletionDetector::classify(
  const FrontierPlanObservation & observation) const
{
  if (
    observation.planning_status == "no_frontiers" &&
    observation.detected_frontiers == 0U &&
    observation.reachable_frontiers == 0U)
  {
    return FrontierExhaustionKind::NoFrontiers;
  }

  if (
    observation.planning_status == "no_reachable_frontiers" &&
    observation.detected_frontiers > 0U &&
    observation.reachable_frontiers == 0U)
  {
    return FrontierExhaustionKind::NoReachableFrontiers;
  }

  if (
    observation.planning_status == "no_selectable_frontier" &&
    observation.detected_frontiers > 0U)
  {
    return FrontierExhaustionKind::NoSelectableFrontier;
  }

  return FrontierExhaustionKind::None;
}

bool FrontierCompletionDetector::allowed(
  const FrontierExhaustionKind kind) const
{
  switch (kind) {
    case FrontierExhaustionKind::None:
      return false;
    case FrontierExhaustionKind::NoFrontiers:
      return config_.allow_no_frontiers;
    case FrontierExhaustionKind::NoReachableFrontiers:
      return config_.allow_no_reachable_frontiers;
    case FrontierExhaustionKind::NoSelectableFrontier:
      return config_.allow_no_selectable_frontier;
  }

  return false;
}

void FrontierCompletionDetector::reset_candidate(std::string reason)
{
  snapshot_.candidate = false;
  snapshot_.confirmed = false;
  snapshot_.exhaustion_kind = FrontierExhaustionKind::None;
  snapshot_.observations = 0U;
  snapshot_.stable_duration_s = 0.0;
  snapshot_.reason = std::move(reason);
  candidate_started_at_s_ = 0.0;
  candidate_map_generation_ = 0U;
  candidate_planned_map_generation_ = 0U;
}

void FrontierCompletionDetector::update_freshness(const double now_s)
{
  snapshot_.status_fresh =
    snapshot_.status_received &&
    now_s - last_received_at_s_ <= config_.status_timeout_s;

  if (!snapshot_.status_fresh && !snapshot_.confirmed) {
    reset_candidate("frontier_status_stale");
  }
}

}  // namespace savo_mapping::autonomous
