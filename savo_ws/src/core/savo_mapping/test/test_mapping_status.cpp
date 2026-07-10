#include "savo_mapping/mapping_status.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(MappingStatusContract, DefaultStatusIsSafeIdle)
{
  const auto status = savo_mapping::make_default_status();

  EXPECT_EQ(status.mode, savo_mapping::MappingMode::MonitorOnly);
  EXPECT_EQ(status.exploration_mode, savo_mapping::ExplorationMode::Idle);
  EXPECT_EQ(status.workflow_phase, savo_mapping::WorkflowPhase::Idle);
  EXPECT_EQ(status.session_state, savo_mapping::SessionState::Idle);

  EXPECT_TRUE(status.healthy);
  EXPECT_FALSE(status.ready);
  EXPECT_FALSE(status.slam_active);
  EXPECT_FALSE(status.map_received);
  EXPECT_FALSE(status.scan_received);
  EXPECT_FALSE(status.tf_ok);
  EXPECT_FALSE(status.odom_ok);

  EXPECT_DOUBLE_EQ(status.quality_score, 0.0);
  EXPECT_EQ(status.heartbeat_seq, 0u);
  EXPECT_TRUE(status.active_map_name.empty());
  EXPECT_EQ(status.message, "idle");
}

TEST(MappingStatusContract, DetectsCoreSensorInputs)
{
  savo_mapping::MappingStatus status;

  EXPECT_FALSE(savo_mapping::has_core_sensor_inputs(status));

  status.scan_received = true;
  EXPECT_TRUE(savo_mapping::has_core_sensor_inputs(status));
}

TEST(MappingStatusContract, DetectsCoreLocalizationInputs)
{
  savo_mapping::MappingStatus status;

  status.tf_ok = true;
  EXPECT_FALSE(savo_mapping::has_core_localization_inputs(status));

  status.odom_ok = true;
  EXPECT_TRUE(savo_mapping::has_core_localization_inputs(status));
}

TEST(MappingStatusContract, DetectsMapData)
{
  savo_mapping::MappingStatus status;

  EXPECT_FALSE(savo_mapping::has_map_data(status));

  status.map_received = true;
  EXPECT_TRUE(savo_mapping::has_map_data(status));
}

TEST(MappingStatusContract, MappingReadyRequiresAllCoreInputs)
{
  savo_mapping::MappingStatus status;

  status.healthy = true;
  status.ready = true;
  status.slam_active = true;
  status.scan_received = true;
  status.tf_ok = true;
  status.odom_ok = true;
  status.map_received = true;
  status.quality_score = 0.80;

  EXPECT_TRUE(savo_mapping::is_mapping_ready(status));

  status.scan_received = false;
  EXPECT_FALSE(savo_mapping::is_mapping_ready(status));

  status.scan_received = true;
  status.tf_ok = false;
  EXPECT_FALSE(savo_mapping::is_mapping_ready(status));

  status.tf_ok = true;
  status.ready = false;
  EXPECT_FALSE(savo_mapping::is_mapping_ready(status));
}

TEST(MappingStatusContract, NavigationHandoffRequiresMapNameAndQuality)
{
  savo_mapping::MappingStatus status;

  status.healthy = true;
  status.ready = true;
  status.slam_active = true;
  status.scan_received = true;
  status.tf_ok = true;
  status.odom_ok = true;
  status.map_received = true;
  status.quality_score = 0.85;

  EXPECT_FALSE(savo_mapping::is_navigation_handoff_ready(status, 0.80));

  status.active_map_name = "savonia_floor_1";
  EXPECT_TRUE(savo_mapping::is_navigation_handoff_ready(status, 0.80));
  EXPECT_FALSE(savo_mapping::is_navigation_handoff_ready(status, 0.90));
}

TEST(MappingStatusContract, DetectsInconsistentStatus)
{
  savo_mapping::MappingStatus status;

  EXPECT_TRUE(savo_mapping::is_status_consistent(status));

  status.ready = true;
  status.healthy = false;
  EXPECT_FALSE(savo_mapping::is_status_consistent(status));

  status.healthy = true;
  status.slam_active = false;
  EXPECT_FALSE(savo_mapping::is_status_consistent(status));

  status.ready = false;
  status.session_state = savo_mapping::SessionState::Saved;
  status.workflow_phase = savo_mapping::WorkflowPhase::Mapping;
  EXPECT_FALSE(savo_mapping::is_status_consistent(status));
}

TEST(MappingStatusContract, ReadinessTextExplainsFirstMissingCondition)
{
  savo_mapping::MappingStatus status;

  status.healthy = false;
  EXPECT_EQ(savo_mapping::readiness_text(status), "not_ready: unhealthy");

  status.healthy = true;
  EXPECT_EQ(savo_mapping::readiness_text(status), "not_ready: slam_inactive");

  status.slam_active = true;
  EXPECT_EQ(savo_mapping::readiness_text(status), "not_ready: missing_scan");

  status.scan_received = true;
  EXPECT_EQ(savo_mapping::readiness_text(status), "not_ready: tf_not_ok");

  status.tf_ok = true;
  EXPECT_EQ(savo_mapping::readiness_text(status), "not_ready: odom_not_ok");

  status.odom_ok = true;
  EXPECT_EQ(savo_mapping::readiness_text(status), "not_ready: missing_map");

  status.map_received = true;
  EXPECT_EQ(savo_mapping::readiness_text(status), "not_ready");

  status.ready = true;
  EXPECT_EQ(savo_mapping::readiness_text(status), "ready");
}

TEST(MappingStatusContract, StatusJsonContainsStableFields)
{
  savo_mapping::MappingStatus status;

  status.mode = savo_mapping::MappingMode::Scan360;
  status.exploration_mode = savo_mapping::ExplorationMode::Scan360;
  status.workflow_phase = savo_mapping::WorkflowPhase::Scan360;
  status.session_state = savo_mapping::SessionState::Active;
  status.healthy = true;
  status.ready = true;
  status.slam_active = true;
  status.map_received = true;
  status.scan_received = true;
  status.tf_ok = true;
  status.odom_ok = true;
  status.quality_score = 0.91;
  status.heartbeat_seq = 42;
  status.active_map_name = "savonia_floor_1";
  status.message = "scan360 running";

  const std::string json = savo_mapping::make_status_json(status);

  EXPECT_NE(json.find("\"mode\":\"scan360\""), std::string::npos);
  EXPECT_NE(json.find("\"exploration_mode\":\"scan360\""), std::string::npos);
  EXPECT_NE(json.find("\"workflow_phase\":\"scan360\""), std::string::npos);
  EXPECT_NE(json.find("\"session_state\":\"active\""), std::string::npos);
  EXPECT_NE(json.find("\"healthy\":true"), std::string::npos);
  EXPECT_NE(json.find("\"ready\":true"), std::string::npos);
  EXPECT_NE(json.find("\"slam_active\":true"), std::string::npos);
  EXPECT_NE(json.find("\"quality_score\":0.91"), std::string::npos);
  EXPECT_NE(json.find("\"heartbeat_seq\":42"), std::string::npos);
  EXPECT_NE(json.find("\"active_map_name\":\"savonia_floor_1\""), std::string::npos);
  EXPECT_NE(json.find("\"message\":\"scan360 running\""), std::string::npos);
}

TEST(MappingStatusContract, StatusJsonEscapesTextFields)
{
  savo_mapping::MappingStatus status;

  status.active_map_name = "map\"one";
  status.message = "line1\nline2";

  const std::string json = savo_mapping::make_status_json(status);

  EXPECT_NE(json.find("\"active_map_name\":\"map\\\"one\""), std::string::npos);
  EXPECT_NE(json.find("\"message\":\"line1\\nline2\""), std::string::npos);
}
