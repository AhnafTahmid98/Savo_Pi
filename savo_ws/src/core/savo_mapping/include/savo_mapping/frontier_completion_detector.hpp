#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace savo_mapping::autonomous
{

enum class FrontierExhaustionKind : std::uint8_t
{
  None = 0,
  NoFrontiers = 1,
  NoReachableFrontiers = 2,
  NoSelectableFrontier = 3,
};

struct FrontierCompletionConfig
{
  std::uint32_t minimum_observations{3};
  double minimum_stable_duration_s{5.0};
  double status_timeout_s{3.0};
  bool allow_no_frontiers{true};
  bool allow_no_reachable_frontiers{true};
  bool allow_no_selectable_frontier{false};
};

struct FrontierPlanObservation
{
  bool received{false};
  bool runtime_enabled{false};
  bool goal_pending{false};
  bool handoff_active{false};

  std::uint64_t map_generation{0};
  std::uint64_t planned_map_generation{0};
  std::uint64_t plan_sequence{0};

  std::string planning_status{"unavailable"};
  std::uint32_t detected_frontiers{0};
  std::uint32_t reachable_frontiers{0};
  double received_at_s{0.0};
};

struct FrontierCompletionSnapshot
{
  bool status_received{false};
  bool status_fresh{false};
  bool candidate{false};
  bool confirmed{false};

  FrontierExhaustionKind exhaustion_kind{FrontierExhaustionKind::None};
  std::uint32_t observations{0};
  double stable_duration_s{0.0};

  std::uint64_t map_generation{0};
  std::uint64_t planned_map_generation{0};
  std::uint64_t plan_sequence{0};
  std::uint32_t detected_frontiers{0};
  std::uint32_t reachable_frontiers{0};

  std::string planning_status{"unavailable"};
  std::string reason{"frontier_status_unavailable"};
};

std::string_view to_string(FrontierExhaustionKind kind);
std::string validate_frontier_completion_config(
  const FrontierCompletionConfig & config);

class FrontierCompletionDetector
{
public:
  explicit FrontierCompletionDetector(
    FrontierCompletionConfig config = FrontierCompletionConfig{});

  FrontierCompletionSnapshot observe(
    const FrontierPlanObservation & observation,
    double now_s);

  FrontierCompletionSnapshot tick(double now_s);

  void reset(std::string reason = "completion_detector_reset");

  const FrontierCompletionSnapshot & snapshot() const noexcept;

private:
  FrontierExhaustionKind classify(
    const FrontierPlanObservation & observation) const;

  bool allowed(FrontierExhaustionKind kind) const;
  void reset_candidate(std::string reason);
  void update_freshness(double now_s);

  FrontierCompletionConfig config_;
  FrontierCompletionSnapshot snapshot_;
  double candidate_started_at_s_{0.0};
  double last_received_at_s_{0.0};
  std::uint64_t candidate_map_generation_{0};
  std::uint64_t candidate_planned_map_generation_{0};
};

}  // namespace savo_mapping::autonomous
