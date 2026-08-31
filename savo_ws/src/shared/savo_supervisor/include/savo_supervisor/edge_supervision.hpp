// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>
#include <vector>

#include "savo_supervisor/supervisor_state.hpp"

namespace savo_supervisor
{

struct BridgeObservation
{
  bool received{false};
  bool fresh{false};
  bool valid{false};
  bool process_alive{false};
  bool bridge_ready{false};
  bool readiness_asserted{false};
  bool heartbeat_fresh{false};
  bool commands_enabled{false};
  bool dds_active{false};
  bool core_visible{false};
  bool edge_visible{false};
  bool stop_ready{false};
  bool teleop_ready{false};
  bool navigation_ready{false};
  std::string reason{"bridge_not_observed"};
};

struct RealSenseObservation
{
  bool received{false};
  bool fresh{false};
  bool valid{false};
  bool healthy{false};
  bool color_ready{false};
  bool depth_ready{false};
  bool pointcloud_ready{false};
  std::string reason{"realsense_not_observed"};
};

struct VoiceObservation
{
  bool received{false};
  bool fresh{false};
  bool valid{false};
  bool ready{false};
  bool heartbeat_fresh{false};
  std::string state{"unknown"};
  std::string reason{"speech_not_observed"};
};

struct VisualOdometryObservation
{
  bool received{false};
  bool fresh{false};
  bool valid{false};
  bool ready{false};
  bool degraded{false};
  std::string state{"unknown"};
  std::string reason{"vo_not_observed"};
};

struct UiObservation
{
  bool enabled{true};
  bool graph_visible{false};
  std::string reason{"ui_node_not_visible"};
};

struct EdgeComponentSummary
{
  std::string name;
  bool enabled{false};
  bool required_for_startup{false};
  bool received{false};
  bool fresh{false};
  bool valid{false};
  bool ready{false};
  bool degraded{false};
  ComponentState state{ComponentState::UNKNOWN};
  std::string reason;
};

struct EdgeCapabilities
{
  bool edge_health_ready{false};
  bool edge_startup_ready{false};
  bool core_edge_link_ready{false};
  bool bridge_ready{false};
  bool remote_command_path_ready{false};
  bool speech_ready{false};
  bool realsense_ready{false};
  bool vo_ready{false};
  bool ui_ready{false};
};

struct EdgeSupervisionState
{
  EdgeCapabilities capabilities{};
  bool degraded{false};
  std::string reason{"edge_not_evaluated"};
  std::vector<EdgeComponentSummary> components;
};

struct EdgeSupervisionPolicy
{
  bool bridge_enabled{true};
  bool bridge_required_for_startup{true};
  bool realsense_enabled{true};
  bool realsense_required_for_startup{false};
  bool speech_enabled{false};
  bool speech_required_for_startup{false};
  bool vo_enabled{true};
  bool vo_required_for_startup{false};
  bool ui_enabled{false};
  bool ui_required_for_startup{false};
};

class EdgeSupervision
{
public:
  explicit EdgeSupervision(EdgeSupervisionPolicy policy = {});

  [[nodiscard]] EdgeSupervisionState Evaluate(
    const BridgeObservation & bridge,
    const RealSenseObservation & realsense,
    const VoiceObservation & speech,
    const VisualOdometryObservation & vo,
    const UiObservation & ui) const;

private:
  EdgeSupervisionPolicy policy_{};
};

}  // namespace savo_supervisor
