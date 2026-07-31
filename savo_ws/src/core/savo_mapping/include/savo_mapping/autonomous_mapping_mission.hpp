#pragma once

#include "savo_mapping/exploration_mode.hpp"
#include "savo_mapping/frontier_completion_detector.hpp"
#include "savo_mapping/mapping_mode.hpp"
#include "savo_mapping/session_state.hpp"
#include "savo_mapping/workflow_phase.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace savo_mapping::autonomous
{

enum class MissionStrategy : std::uint8_t
{
  None = 0,
  Frontier = 1,
  Coverage = 2,
};

enum class MissionState : std::uint8_t
{
  Idle = 0,
  Starting = 1,
  WaitingForAuthority = 2,
  Exploring = 3,
  Pausing = 4,
  Paused = 5,
  Resuming = 6,
  Canceling = 7,
  Saving = 8,
  Verifying = 9,
  Completed = 10,
  Canceled = 11,
  Failed = 12,
  CompletionPending = 13,
  CapturingStartPose = 14,
  InitialScan360 = 15,
  InitialHeadScan = 16,
  ConditionalScan360 = 17,
  CoveragePending = 18,
  Coverage = 19,
  ReturningToStart = 20,
  FinalScan360 = 21,
  FinalHeadScan = 22,
  VerifyingLocations = 23,
  AwaitingApproval = 24,
  Releasing = 25,
};

enum class MissionResult : std::uint8_t
{
  Succeeded = 0,
  InvalidRequest = 1,
  Busy = 2,
  ReadinessLost = 3,
  NavigationFailed = 4,
  TimedOut = 5,
  Canceled = 6,
  SaveFailed = 7,
  QualityRejected = 8,
  InternalError = 9,
  ScanFailed = 10,
  StartPoseUnavailable = 11,
  None = 255,
};

enum class MissionCommand : std::uint8_t
{
  Pause = 1,
  Resume = 2,
  Cancel = 3,
  RequestScan360 = 4,
};

struct MissionRequest
{
  std::string mission_id;
  std::string actor_id;
  std::string map_id;
  std::uint32_t map_revision{0};
  MissionStrategy strategy{MissionStrategy::None};
  bool auto_save{true};
  bool require_quality_approval{true};
};

struct MissionInputs
{
  std::optional<MappingMode> mode;
  std::optional<ExplorationMode> exploration_mode;
  std::optional<WorkflowPhase> workflow_phase;
  std::optional<SessionState> session_state;

  bool readiness_received{false};
  bool mapping_ready{false};

  bool safety_stop_received{false};
  bool safety_stop_active{true};

  bool runtime_authority_received{false};
  bool runtime_authorized{false};

  bool handoff_state_received{false};
  bool handoff_active{false};
  std::string handoff_state{"unavailable"};

  bool require_start_pose_capture{true};
  bool require_initial_scan360{true};
  bool require_initial_head_scan{true};

  std::uint64_t start_pose_generation{0};
  bool start_pose_capture_started{false};
  bool start_pose_capture_complete{false};
  bool start_pose_valid{false};
  std::string start_pose_reason{"start_pose_not_requested"};

  std::uint64_t scan360_generation{0};
  bool scan360_started{false};
  bool scan360_active{false};
  bool scan360_complete{false};
  bool scan360_succeeded{false};
  std::string scan360_state{"idle"};
  std::string scan360_reason{"scan360_not_requested"};

  std::uint64_t head_scan_generation{0};
  bool head_scan_started{false};
  bool head_scan_active{false};
  bool head_scan_paused{false};
  bool head_scan_complete{false};
  bool head_scan_succeeded{false};
  std::string head_scan_state{"idle"};
  std::string head_scan_reason{"head_scan_not_requested"};

  bool frontier_status_received{false};
  bool frontier_status_fresh{false};
  std::string frontier_planning_status{"unavailable"};
  std::uint64_t frontier_plan_sequence{0};
  std::uint64_t frontier_map_generation{0};
  std::uint32_t detected_frontiers{0};
  std::uint32_t reachable_frontiers{0};
  std::uint32_t exhaustion_observations{0};
  double exhaustion_stable_duration_s{0.0};
  bool completion_candidate{false};
  bool completion_confirmed{false};
  std::string completion_reason{"completion_not_observed"};

  bool map_save_started{false};
  bool map_save_complete{false};
  bool map_save_succeeded{false};
  std::string map_save_reason{"map_save_not_started"};
  std::string saved_session_directory;

  bool verification_started{false};
  bool verification_complete{false};
  bool verification_succeeded{false};
  std::string verification_reason{"verification_not_started"};
};

struct MissionSnapshot
{
  MissionRequest request;
  MissionState state{MissionState::Idle};
  MissionResult result{MissionResult::None};

  bool active{false};
  bool runtime_authorized{false};
  bool mapping_ready{false};
  bool safety_stop_active{true};
  bool handoff_active{false};

  std::uint32_t goals_succeeded{0};
  std::uint32_t goals_failed{0};

  bool start_pose_capture_started{false};
  bool start_pose_capture_complete{false};
  bool start_pose_valid{false};
  std::uint64_t start_map_generation{0};
  std::string start_pose_reason{"start_pose_not_requested"};

  bool initial_scan360_complete{false};
  bool initial_scan360_succeeded{false};
  bool initial_head_scan_complete{false};
  bool initial_head_scan_succeeded{false};
  std::uint32_t conditional_scan360_completed{0};

  bool scan360_started{false};
  bool scan360_active{false};
  bool scan360_complete{false};
  bool scan360_succeeded{false};
  std::string scan360_stage{"none"};
  std::string scan360_state{"idle"};
  std::string scan360_reason{"scan360_not_requested"};

  bool head_scan_started{false};
  bool head_scan_active{false};
  bool head_scan_paused{false};
  bool head_scan_complete{false};
  bool head_scan_succeeded{false};
  std::string head_scan_stage{"none"};
  std::string head_scan_state{"idle"};
  std::string head_scan_reason{"head_scan_not_requested"};

  bool frontier_status_received{false};
  bool frontier_status_fresh{false};
  std::string frontier_planning_status{"unavailable"};
  std::uint64_t frontier_plan_sequence{0};
  std::uint64_t frontier_map_generation{0};
  std::uint32_t detected_frontiers{0};
  std::uint32_t reachable_frontiers{0};
  std::uint32_t exhaustion_observations{0};
  double exhaustion_stable_duration_s{0.0};
  bool completion_candidate{false};
  bool completion_confirmed{false};
  std::string completion_reason{"completion_not_observed"};

  bool map_save_started{false};
  bool map_save_complete{false};
  bool map_saved{false};
  std::string map_save_reason{"map_save_not_started"};
  std::string saved_session_directory;

  bool verification_started{false};
  bool verification_complete{false};
  bool map_verified{false};
  std::string verification_reason{"verification_not_started"};

  std::string reason{"idle"};
};

struct MissionDecision
{
  bool accepted{true};
  bool terminal{false};

  bool request_start_session{false};
  bool request_frontier_mode{false};
  bool request_scan360_mode{false};
  bool request_monitor_mode{false};
  bool request_handoff_cancel{false};
  bool request_scan360_start{false};
  bool request_scan360_cancel{false};
  bool request_head_scan_start{false};
  bool request_head_scan_pause{false};
  bool request_head_scan_resume{false};
  bool request_cancel_session{false};
  bool request_start_pose_capture{false};
  bool request_map_save{false};
  bool request_saved_map_verification{false};

  MissionSnapshot snapshot;
  std::string reason{"idle"};
};

std::string_view to_string(MissionStrategy strategy);
std::string_view to_string(MissionState state);
std::string_view to_string(MissionResult result);
std::string_view to_string(MissionCommand command);

bool is_active(MissionState state);
bool is_terminal(MissionState state);
bool is_scan_state(MissionState state);
bool is_valid_request(const MissionRequest & request);
bool is_active_handoff_state(std::string_view state);
bool is_success_handoff_state(std::string_view state);
bool is_failed_handoff_state(std::string_view state);

class AutonomousMappingMission
{
public:
  MissionDecision start(
    const MissionRequest & request,
    const MissionInputs & inputs);

  MissionDecision control(
    MissionCommand command,
    std::string reason,
    const MissionInputs & inputs);

  MissionDecision abort(
    MissionResult result,
    std::string reason,
    const MissionInputs & inputs);

  MissionDecision observe(const MissionInputs & inputs);

  const MissionSnapshot & snapshot() const noexcept;

private:
  MissionDecision evaluate(const MissionInputs & inputs);
  MissionDecision evaluate_start_sequence(const MissionInputs & inputs);
  MissionDecision evaluate_scan360_stage(const MissionInputs & inputs);
  MissionDecision evaluate_head_scan_stage(const MissionInputs & inputs);
  MissionDecision evaluate_frontier_entry(const MissionInputs & inputs);
  MissionDecision fail(
    MissionResult result,
    std::string reason);

  void update_observations(const MissionInputs & inputs);
  void update_handoff_counters(const MissionInputs & inputs);
  bool authority_inputs_complete(const MissionInputs & inputs) const;
  bool safe_mapping_state(const MissionInputs & inputs) const;
  bool workflow_is_frontier(const MissionInputs & inputs) const;
  bool workflow_is_scan360(const MissionInputs & inputs) const;
  bool workflow_is_monitor_only(const MissionInputs & inputs) const;
  bool session_is_active(const MissionInputs & inputs) const;
  bool session_is_terminal(const MissionInputs & inputs) const;

  void begin_scan_stage(
    MissionState state,
    const MissionInputs & inputs,
    std::string reason);
  void begin_head_scan_stage(
    MissionState state,
    const MissionInputs & inputs,
    std::string reason);
  void begin_pause(
    MissionState resume_state,
    std::string reason,
    bool transition_to_conditional_scan);

  MissionDecision rejected(std::string reason) const;
  MissionDecision decision(std::string reason) const;
  void enter(MissionState state, std::string reason);
  void begin_stop(MissionResult result, std::string reason);

  MissionSnapshot snapshot_;
  MissionResult pending_terminal_result_{MissionResult::Canceled};
  std::string pending_terminal_reason_{"mission_canceled"};
  std::string previous_handoff_state_{"unavailable"};

  MissionState resume_state_{MissionState::Exploring};
  bool transition_to_conditional_scan_{false};

  std::uint64_t start_pose_generation_floor_{0};
  std::uint64_t scan360_generation_floor_{0};
  std::uint64_t head_scan_generation_floor_{0};

  bool start_pose_captured_{false};
  bool initial_scan360_completed_{false};
  bool initial_head_scan_completed_{false};
  std::uint32_t conditional_scan360_completed_{0};
};

}  // namespace savo_mapping::autonomous
