// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_supervisor/edge_supervision.hpp"

namespace
{

savo_supervisor::BridgeObservation healthy_bridge()
{
  savo_supervisor::BridgeObservation value;
  value.received = true;
  value.fresh = true;
  value.valid = true;
  value.process_alive = true;
  value.bridge_ready = true;
  value.readiness_asserted = true;
  value.heartbeat_fresh = true;
  value.commands_enabled = true;
  value.dds_active = true;
  value.core_visible = true;
  value.edge_visible = true;
  value.stop_ready = true;
  value.teleop_ready = true;
  value.navigation_ready = true;
  value.reason = "bridge_ready";
  return value;
}

savo_supervisor::RealSenseObservation healthy_camera()
{
  savo_supervisor::RealSenseObservation value;
  value.received = true;
  value.fresh = true;
  value.valid = true;
  value.healthy = true;
  value.color_ready = true;
  value.depth_ready = true;
  value.pointcloud_ready = true;
  value.reason = "camera_ready";
  return value;
}

savo_supervisor::VoiceObservation healthy_speech()
{
  savo_supervisor::VoiceObservation value;
  value.received = true;
  value.fresh = true;
  value.valid = true;
  value.ready = true;
  value.heartbeat_fresh = true;
  value.reason = "speech_ready";
  return value;
}

savo_supervisor::VisualOdometryObservation healthy_vo()
{
  savo_supervisor::VisualOdometryObservation value;
  value.received = true;
  value.fresh = true;
  value.valid = true;
  value.ready = true;
  value.reason = "vo_ready";
  return value;
}

savo_supervisor::UiObservation healthy_ui()
{
  savo_supervisor::UiObservation value;
  value.enabled = true;
  value.graph_visible = true;
  value.reason = "ui_node_visible";
  return value;
}

}  // namespace

TEST(EdgeSupervision, HealthyEdgeEnablesStartupAndRemotePath)
{
  const auto result = savo_supervisor::EdgeSupervision{}.Evaluate(
    healthy_bridge(), healthy_camera(), healthy_speech(), healthy_vo(), healthy_ui());
  EXPECT_TRUE(result.capabilities.edge_health_ready);
  EXPECT_TRUE(result.capabilities.edge_startup_ready);
  EXPECT_TRUE(result.capabilities.core_edge_link_ready);
  EXPECT_TRUE(result.capabilities.remote_command_path_ready);
  EXPECT_FALSE(result.degraded);
}

TEST(EdgeSupervision, OptionalSpeechLossDegradesWithoutBlockingStartup)
{
  auto speech = healthy_speech();
  speech.ready = false;
  speech.reason = "speech_error";
  const auto result = savo_supervisor::EdgeSupervision{}.Evaluate(
    healthy_bridge(), healthy_camera(), speech, healthy_vo(), healthy_ui());
  EXPECT_FALSE(result.capabilities.edge_health_ready);
  EXPECT_TRUE(result.capabilities.edge_startup_ready);
  EXPECT_TRUE(result.degraded);
}

TEST(EdgeSupervision, RequiredBridgeLossBlocksStartup)
{
  auto bridge = healthy_bridge();
  bridge.core_visible = false;
  bridge.reason = "graph_evidence_incomplete";
  const auto result = savo_supervisor::EdgeSupervision{}.Evaluate(
    bridge, healthy_camera(), healthy_speech(), healthy_vo(), healthy_ui());
  EXPECT_FALSE(result.capabilities.edge_startup_ready);
  EXPECT_FALSE(result.capabilities.remote_command_path_ready);
  EXPECT_EQ(result.reason, "edge_startup_dependency_unavailable");
}

TEST(EdgeSupervision, CameraCanBeConfiguredAsStartupRequirement)
{
  savo_supervisor::EdgeSupervisionPolicy policy;
  policy.realsense_required_for_startup = true;
  auto camera = healthy_camera();
  camera.healthy = false;
  const auto result = savo_supervisor::EdgeSupervision{policy}.Evaluate(
    healthy_bridge(), camera, healthy_speech(), healthy_vo(), healthy_ui());
  EXPECT_FALSE(result.capabilities.edge_startup_ready);
}

TEST(EdgeSupervision, StopTransportIsRequiredForRemoteCommandPath)
{
  auto bridge = healthy_bridge();
  bridge.stop_ready = false;
  bridge.reason = "stop_transport_not_ready";
  const auto result = savo_supervisor::EdgeSupervision{}.Evaluate(
    bridge, healthy_camera(), healthy_speech(), healthy_vo(), healthy_ui());
  EXPECT_FALSE(result.capabilities.edge_startup_ready);
  EXPECT_FALSE(result.capabilities.remote_command_path_ready);
}
