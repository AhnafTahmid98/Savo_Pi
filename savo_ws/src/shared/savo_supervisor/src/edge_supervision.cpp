// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/edge_supervision.hpp"

#include <string>
#include <utility>

namespace savo_supervisor
{
namespace
{

EdgeComponentSummary disabled(const std::string & name)
{
  EdgeComponentSummary result;
  result.name = name;
  result.state = ComponentState::DISABLED;
  result.ready = true;
  result.valid = true;
  result.fresh = true;
  result.reason = name + "_disabled";
  return result;
}

EdgeComponentSummary summarize(
  const std::string & name,
  const bool enabled,
  const bool required,
  const bool received,
  const bool fresh,
  const bool valid,
  const bool ready,
  const bool degraded,
  const std::string & reason)
{
  if (!enabled) {
    return disabled(name);
  }
  EdgeComponentSummary result;
  result.name = name;
  result.enabled = true;
  result.required_for_startup = required;
  result.received = received;
  result.fresh = fresh;
  result.valid = valid;
  result.ready = ready;
  result.degraded = degraded;
  result.reason = reason;
  if (!received) {
    result.state = ComponentState::STALE;
  } else if (!fresh) {
    result.state = ComponentState::STALE;
  } else if (!valid) {
    result.state = ComponentState::INVALID;
  } else if (ready) {
    result.state = degraded ? ComponentState::DEGRADED : ComponentState::OK;
  } else {
    result.state = degraded ? ComponentState::DEGRADED : ComponentState::ERROR;
  }
  return result;
}

bool required_ready(const EdgeComponentSummary & component)
{
  return !component.enabled || !component.required_for_startup || component.ready;
}

bool component_healthy(const EdgeComponentSummary & component)
{
  return !component.enabled || component.ready;
}

}  // namespace

EdgeSupervision::EdgeSupervision(EdgeSupervisionPolicy policy)
: policy_(std::move(policy))
{
}

EdgeSupervisionState EdgeSupervision::Evaluate(
  const BridgeObservation & bridge,
  const RealSenseObservation & realsense,
  const VoiceObservation & speech,
  const VisualOdometryObservation & vo,
  const UiObservation & ui) const
{
  EdgeSupervisionState result;
  const bool link_ready = bridge.valid && bridge.fresh && bridge.dds_active &&
    bridge.core_visible && bridge.edge_visible;
  const bool bridge_ready = bridge.valid && bridge.fresh && bridge.heartbeat_fresh &&
    bridge.readiness_asserted && bridge.process_alive && bridge.bridge_ready &&
    bridge.commands_enabled && bridge.stop_ready && link_ready;
  const bool realsense_ready = realsense.valid && realsense.fresh && realsense.healthy &&
    realsense.color_ready && realsense.depth_ready && realsense.pointcloud_ready;
  const bool speech_ready = speech.valid && speech.fresh && speech.heartbeat_fresh && speech.ready;
  const bool vo_ready = vo.valid && vo.fresh && vo.ready;
  const bool ui_ready = !policy_.ui_enabled || (ui.enabled && ui.graph_visible);

  result.components.push_back(summarize(
    "bridge", policy_.bridge_enabled, policy_.bridge_required_for_startup,
    bridge.received, bridge.fresh && bridge.heartbeat_fresh, bridge.valid,
    bridge_ready, false, bridge.reason));
  result.components.push_back(summarize(
    "realsense", policy_.realsense_enabled, policy_.realsense_required_for_startup,
    realsense.received, realsense.fresh, realsense.valid, realsense_ready,
    realsense.valid && realsense.fresh && !realsense_ready, realsense.reason));
  result.components.push_back(summarize(
    "speech", policy_.speech_enabled, policy_.speech_required_for_startup,
    speech.received, speech.fresh && speech.heartbeat_fresh, speech.valid,
    speech_ready, speech.valid && speech.fresh && !speech_ready, speech.reason));
  result.components.push_back(summarize(
    "vo", policy_.vo_enabled, policy_.vo_required_for_startup,
    vo.received, vo.fresh, vo.valid, vo_ready, vo.degraded, vo.reason));
  result.components.push_back(summarize(
    "ui", policy_.ui_enabled, policy_.ui_required_for_startup,
    ui.graph_visible, ui.graph_visible, true, ui_ready, false, ui.reason));

  result.capabilities.core_edge_link_ready = policy_.bridge_enabled && link_ready;
  result.capabilities.bridge_ready = !policy_.bridge_enabled || bridge_ready;
  result.capabilities.realsense_ready = !policy_.realsense_enabled || realsense_ready;
  result.capabilities.speech_ready = !policy_.speech_enabled || speech_ready;
  result.capabilities.vo_ready = !policy_.vo_enabled || vo_ready;
  result.capabilities.ui_ready = ui_ready;

  result.capabilities.edge_startup_ready = true;
  result.capabilities.edge_health_ready = true;
  for (const auto & component : result.components) {
    result.capabilities.edge_startup_ready =
      result.capabilities.edge_startup_ready && required_ready(component);
    result.capabilities.edge_health_ready =
      result.capabilities.edge_health_ready && component_healthy(component);
  }
  result.capabilities.remote_command_path_ready = bridge_ready;
  result.degraded = !result.capabilities.edge_health_ready;

  if (!result.capabilities.edge_startup_ready) {
    result.reason = "edge_startup_dependency_unavailable";
  } else if (result.degraded) {
    result.reason = "edge_optional_capability_degraded";
  } else {
    result.reason = "edge_operational";
  }
  return result;
}

}  // namespace savo_supervisor
