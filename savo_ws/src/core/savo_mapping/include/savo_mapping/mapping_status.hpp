#pragma once

#include "savo_mapping/exploration_mode.hpp"
#include "savo_mapping/mapping_mode.hpp"
#include "savo_mapping/mapping_types.hpp"
#include "savo_mapping/session_state.hpp"
#include "savo_mapping/workflow_phase.hpp"

#include <cstdint>
#include <string>

namespace savo_mapping
{

struct MappingStatus
{
  MappingMode mode{MappingMode::MonitorOnly};
  ExplorationMode exploration_mode{ExplorationMode::Idle};
  WorkflowPhase workflow_phase{WorkflowPhase::Idle};
  SessionState session_state{SessionState::Idle};

  bool healthy{true};
  bool ready{false};
  bool slam_active{false};

  bool map_received{false};
  bool scan_received{false};
  bool tf_ok{false};
  bool odom_ok{false};

  double quality_score{0.0};
  std::uint64_t heartbeat_seq{0};

  std::string active_map_name{};
  std::string message{"idle"};
};

MappingStatus make_default_status();

bool has_core_sensor_inputs(const MappingStatus & status);
bool has_core_localization_inputs(const MappingStatus & status);
bool has_map_data(const MappingStatus & status);
bool is_mapping_ready(const MappingStatus & status);
bool is_navigation_handoff_ready(const MappingStatus & status, double quality_threshold);
bool is_status_consistent(const MappingStatus & status);

std::string readiness_text(const MappingStatus & status);
std::string make_status_json(const MappingStatus & status);

}  // namespace savo_mapping
