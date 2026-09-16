// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>
#include <chrono>
#include <string>
#include "savo_mapping/local_mapping_health.hpp"

namespace
{
using savo_mapping::local_health::Monitor;
using Clock = std::chrono::steady_clock;

void feed(Monitor & monitor, Clock::time_point now)
{
  monitor.observe("base", R"({"status_level":"OK","backend":{"connected":true},)"
    R"("diagnostics":{"last_board_error":""}})", now);
  for (const auto * source : {"control", "mux", "shaper"}) {
    monitor.observe(source, "mode=STOP;safety_stop=false;external_stop=false;recovery_active=false",
      now);
  }
  monitor.observe("lidar", R"({"status":"OK","driver_running":true,"hardware_ok":true})", now);
  monitor.observe("lidar_heartbeat", R"({"status":"OK","driver_running":true})", now);
  for (const auto * source : {"localization", "localization_summary", "localization_heartbeat"}) {
    monitor.observe(source, R"({"schema_version":1,"state":"OK","ready":true,)"
      R"("degraded":false,"alive":true,"reason_code":"ok","stamp_s":10.0})",
      now);
  }
  monitor.observe("perception", R"({"overall_ok":true,"overall_status":"OK"})", now);
  monitor.observe("perception_safety",
    R"({"active_decision":{"stop_required":false,"slowdown_factor":1.0}})", now);
  monitor.observe("perception_heartbeat", R"({"ok":true})", now);
  monitor.observe("base_battery", "Base battery OK: 8.10 V", now);
  monitor.observe("core_ups", "Core UPS OK: 4.00 V", now);
  monitor.observe("safety_stop", "false", now);
  monitor.observe("safety_slowdown", "1.0", now);
  monitor.observe("nav", "state=blocked;goal_acceptance_allowed=false;"
    "reason=control_mode_not_navigation;failed_dependencies=control_mode_permission", now);
  monitor.observe("slam", R"({"service_available":true,"response_received":true,)"
    R"("response_fresh":true,"healthy":true,"state_id":3})", now);
}
}  // namespace

TEST(LocalMappingHealth, DirectHealthySourcesAdmitInStopWithoutOptionalEdge)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  EXPECT_FALSE(monitor.evaluate(true, now).admission_ready);
  feed(monitor, now);
  EXPECT_TRUE(monitor.evaluate(true, now).admission_ready);
  EXPECT_TRUE(monitor.evaluate(true, now).continuation_ready);
  EXPECT_FALSE(monitor.evaluate(false, now).continuation_ready);
}

TEST(LocalMappingHealth, RequiredFailureAndExpiredObservationFailClosed)
{
  const auto now = Clock::time_point{};
  for (const auto * source : {"base", "control", "mux", "shaper", "lidar", "lidar_heartbeat",
      "localization", "localization_summary", "localization_heartbeat", "perception",
      "perception_safety", "perception_heartbeat", "base_battery", "core_ups", "safety_stop",
      "safety_slowdown", "nav", "slam"})
  {
    Monitor monitor;
    feed(monitor, now);
    monitor.observe(source, "invalid", now);
    EXPECT_FALSE(monitor.evaluate(true, now).continuation_ready) << source;
  }
  Monitor monitor;
  feed(monitor, now);
  EXPECT_FALSE(monitor.evaluate(true, now + std::chrono::milliseconds(1001)).continuation_ready);
}

TEST(LocalMappingHealth, EnvironmentalStopAndLowPowerPermitOnlyContinuation)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  feed(monitor, now);
  monitor.observe("safety_stop", "true", now);
  monitor.observe("nav", "state=blocked;goal_acceptance_allowed=false;reason=safety_stop_active;"
    "failed_dependencies=safety_stop,control_mode_permission", now);
  EXPECT_FALSE(monitor.evaluate(true, now).admission_ready);
  EXPECT_TRUE(monitor.evaluate(true, now).continuation_ready);
  feed(monitor, now);
  monitor.observe("base_battery", "Base battery LOW: 7.00 V", now);
  EXPECT_FALSE(monitor.evaluate(true, now).admission_ready);
  EXPECT_TRUE(monitor.evaluate(true, now).continuation_ready);
  for (const auto * source : {"base_battery", "core_ups"}) {
    feed(monitor, now);
    monitor.observe(source, std::string("{\"source\":\"") + source +
      "\",\"state\":\"CRITICAL\",\"voltage_v\":3.0}", now);
    EXPECT_FALSE(monitor.evaluate(true, now).continuation_ready);
  }
}

TEST(LocalMappingHealth, NavInfrastructureFailureCannotBeHiddenByObstacle)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  feed(monitor, now);
  monitor.observe("safety_stop", "true", now);
  monitor.observe("nav", "state=blocked;goal_acceptance_allowed=false;reason=safety_stop_active;"
    "failed_dependencies=safety_stop,nav2_action_server", now);
  EXPECT_FALSE(monitor.evaluate(true, now).continuation_ready);
}

TEST(LocalMappingHealth, ProductionControlWhitespaceDoesNotHideSafetyInterlock)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  feed(monitor, now);
  monitor.observe(
    "control",
    "mode=STOP; previous=NAV; reason=safety_stop; source=perception; "
    "safety_stop=true; external_stop=false; recovery_active=false; "
    "manual_override=false; request_stale=false; mux_mode=STOP",
    now);

  const auto decision = monitor.evaluate(true, now);
  EXPECT_FALSE(decision.admission_ready);
  EXPECT_TRUE(decision.continuation_ready);
}

TEST(LocalMappingHealth, NavEnvironmentalBlockerRestrictsAdmissionOnly)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  feed(monitor, now);
  monitor.observe(
    "nav",
    "state=blocked;goal_acceptance_allowed=false;reason=safety_stop_active;"
    "failed_dependencies=safety_stop,control_mode_permission",
    now);

  const auto decision = monitor.evaluate(true, now);
  EXPECT_FALSE(decision.admission_ready);
  EXPECT_TRUE(decision.continuation_ready);
}

TEST(LocalMappingHealth, LocalizationRequiresItsFullProductionContract)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  feed(monitor, now);
  monitor.observe(
    "localization_summary",
    R"({"schema_version":1,"state":"OK","ready":true,"stamp_s":11.0})",
    now);
  EXPECT_FALSE(monitor.evaluate(true, now).continuation_ready);

  feed(monitor, now);
  monitor.observe(
    "localization",
    R"({"schema_version":1,"state":"OK","ready":true,"degraded":false,)"
    R"("reason_code":"ok","stamp_s":11.0,"stamp_s":12.0})",
    now);
  EXPECT_FALSE(monitor.evaluate(true, now).continuation_ready);
}

TEST(LocalMappingHealth, LocalizationStreamsMustRemainConsistent)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  feed(monitor, now);
  monitor.observe(
    "localization_summary",
    R"({"schema_version":1,"state":"DEGRADED","ready":true,)"
    R"("degraded":true,"reason_code":"reduced_rate","stamp_s":10.0})",
    now);

  const auto decision = monitor.evaluate(true, now);
  EXPECT_FALSE(decision.admission_ready);
  EXPECT_FALSE(decision.continuation_ready);
  EXPECT_EQ(decision.reason, "localization_streams_inconsistent");
}

TEST(LocalMappingHealth, AdjacentReadyLocalizationSnapshotsAreNotContradictory)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  feed(monitor, now);
  monitor.observe("localization_summary",
    R"({"schema_version":1,"state":"DEGRADED","ready":true,)"
    R"("degraded":true,"reason_code":"optional_vo_stale","stamp_s":11.0})", now);
  EXPECT_TRUE(monitor.evaluate(true, now).continuation_ready);
  EXPECT_TRUE(monitor.evaluate(true, now).admission_ready);
}

TEST(LocalMappingHealth, ReceiptAndPayloadTimeRegressionFailClosed)
{
  using namespace std::chrono_literals;
  const auto now = Clock::time_point{};

  Monitor receipt_regression;
  feed(receipt_regression, now + 1s);
  receipt_regression.observe(
    "perception_heartbeat", R"({"ok":true})", now);
  EXPECT_FALSE(receipt_regression.evaluate(true, now + 1s).continuation_ready);

  Monitor payload_regression;
  feed(payload_regression, now);
  payload_regression.observe(
    "localization",
    R"({"schema_version":1,"state":"OK","ready":true,"degraded":false,)"
    R"("reason_code":"ok","stamp_s":9.0})",
    now + 1ms);
  EXPECT_FALSE(payload_regression.evaluate(true, now + 1ms).continuation_ready);
}

TEST(LocalMappingHealth, SemanticReadinessTracksOptionalHeadAndLocationsEvidence)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  feed(monitor, now);
  monitor.observe(
    "head",
    R"({"operational":true,"pan_tilt_ready":true,"camera_ready":true,)"
    R"("camera_pose_ready":true})",
    now);
  monitor.observe(
    "locations",
    R"({"component":"savo_locations","read_ready":true,"write_ready":true,)"
    R"("storage_healthy":true,"mutation_in_progress":false})",
    now);

  EXPECT_TRUE(monitor.evaluate(true, now).semantic_ready);
  EXPECT_FALSE(monitor.evaluate(false, now).semantic_ready);

  monitor.observe("head", "{}", now);
  const auto invalid_head = monitor.evaluate(true, now);
  EXPECT_TRUE(invalid_head.admission_ready);
  EXPECT_TRUE(invalid_head.continuation_ready);
  EXPECT_FALSE(invalid_head.semantic_ready);
  EXPECT_EQ(invalid_head.reason, "head_invalid");
}

TEST(LocalMappingHealth, EdgeUpsIsOptionalUnlessExplicitlyExpected)
{
  const auto now = Clock::time_point{};
  Monitor optional_edge;
  feed(optional_edge, now);
  EXPECT_TRUE(optional_edge.evaluate(true, now).admission_ready);

  Monitor required_edge(true);
  feed(required_edge, now);
  EXPECT_FALSE(required_edge.evaluate(true, now).continuation_ready);
  required_edge.observe("edge_ups", "Edge UPS OK: 4.10 V", now);
  EXPECT_TRUE(required_edge.evaluate(true, now).admission_ready);
  required_edge.observe("edge_ups", "Edge UPS LOW: 3.30 V", now);
  EXPECT_FALSE(required_edge.evaluate(true, now).admission_ready);
  EXPECT_TRUE(required_edge.evaluate(true, now).continuation_ready);
}

TEST(LocalMappingHealth, PublishedPayloadHasStableSchemaAndProducerIdentity)
{
  Monitor monitor;
  const auto now = Clock::time_point{};
  feed(monitor, now);
  EXPECT_EQ(
    monitor.evaluate(true, now).json(),
    R"({"admission_ready":true,"continuation_ready":true,"node":"savo_mapping",)"
    R"("reason":"head_missing_stale_or_invalid","schema_version":1,"semantic_ready":false})");
}
