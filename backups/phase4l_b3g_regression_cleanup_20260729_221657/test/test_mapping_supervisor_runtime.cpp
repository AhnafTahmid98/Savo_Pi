#include "savo_mapping/map_quality_placeholder.hpp"
#include "savo_mapping/supervisor_runtime.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

TEST(SupervisorRuntimeContract, FreshnessRequiresValidAgeAndTimeout)
{
  EXPECT_TRUE(savo_mapping::is_fresh_age(0.0, 1.0));
  EXPECT_TRUE(savo_mapping::is_fresh_age(1.0, 1.0));

  EXPECT_FALSE(savo_mapping::is_fresh_age(1.01, 1.0));
  EXPECT_FALSE(savo_mapping::is_fresh_age(-0.1, 1.0));
  EXPECT_FALSE(savo_mapping::is_fresh_age(0.1, 0.0));

  EXPECT_FALSE(
    savo_mapping::is_fresh_age(
      std::numeric_limits<double>::infinity(),
      1.0));

  EXPECT_FALSE(
    savo_mapping::is_fresh_age(
      0.1,
      std::numeric_limits<double>::quiet_NaN()));
}

TEST(SupervisorRuntimeContract, AllRequiredInputsProduceReady)
{
  const savo_mapping::SupervisorRequirements requirements;

  const savo_mapping::SupervisorInputs inputs{
    true,
    true,
    true,
    true,
    true
  };

  const auto decision =
    savo_mapping::evaluate_supervisor_runtime(
    requirements,
    inputs);

  EXPECT_TRUE(decision.ready);
  EXPECT_TRUE(decision.slam_active);
  EXPECT_EQ(decision.reason, "ready");
}

TEST(SupervisorRuntimeContract, MissingScanHasHighestInputPriority)
{
  const savo_mapping::SupervisorRequirements requirements;

  const savo_mapping::SupervisorInputs inputs{
    false,
    false,
    false,
    false,
    false
  };

  const auto decision =
    savo_mapping::evaluate_supervisor_runtime(
    requirements,
    inputs);

  EXPECT_FALSE(decision.ready);
  EXPECT_FALSE(decision.slam_active);
  EXPECT_EQ(
    decision.reason,
    "not_ready: missing_or_stale_scan");
}

TEST(SupervisorRuntimeContract, DetectsMissingOdom)
{
  const savo_mapping::SupervisorRequirements requirements;

  const savo_mapping::SupervisorInputs inputs{
    true,
    true,
    false,
    true,
    true
  };

  const auto decision =
    savo_mapping::evaluate_supervisor_runtime(
    requirements,
    inputs);

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(
    decision.reason,
    "not_ready: missing_or_stale_odom");
}

TEST(SupervisorRuntimeContract, DetectsMissingMap)
{
  const savo_mapping::SupervisorRequirements requirements;

  const savo_mapping::SupervisorInputs inputs{
    true,
    false,
    true,
    true,
    false
  };

  const auto decision =
    savo_mapping::evaluate_supervisor_runtime(
    requirements,
    inputs);

  EXPECT_FALSE(decision.ready);
  EXPECT_FALSE(decision.slam_active);
  EXPECT_EQ(
    decision.reason,
    "not_ready: missing_or_stale_map");
}

TEST(SupervisorRuntimeContract, RejectsInvalidMapStructure)
{
  const savo_mapping::SupervisorRequirements requirements;

  const savo_mapping::SupervisorInputs inputs{
    true,
    true,
    true,
    true,
    false
  };

  const auto decision =
    savo_mapping::evaluate_supervisor_runtime(
    requirements,
    inputs);

  EXPECT_FALSE(decision.ready);
  EXPECT_FALSE(decision.slam_active);
  EXPECT_EQ(
    decision.reason,
    "not_ready: invalid_map_structure");
}

TEST(SupervisorRuntimeContract, IncludesTfFailureReason)
{
  const savo_mapping::SupervisorRequirements requirements;

  const savo_mapping::SupervisorInputs inputs{
    true,
    true,
    true,
    false,
    true
  };

  const auto decision =
    savo_mapping::evaluate_supervisor_runtime(
    requirements,
    inputs,
    "missing_transform:map<-odom");

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(
    decision.reason,
    "not_ready: tf_not_ok:missing_transform:map<-odom");
}

TEST(SupervisorRuntimeContract, OptionalInputsDoNotBlockReadiness)
{
  const savo_mapping::SupervisorRequirements requirements{
    false,
    false,
    false,
    false
  };

  const savo_mapping::SupervisorInputs inputs{
    false,
    false,
    false,
    false,
    false
  };

  const auto decision =
    savo_mapping::evaluate_supervisor_runtime(
    requirements,
    inputs);

  EXPECT_TRUE(decision.ready);
  EXPECT_FALSE(decision.slam_active);
  EXPECT_EQ(decision.reason, "ready");
}

TEST(MapQualityPlaceholderContract, ValidMapProducesStructuralMetrics)
{
  const std::vector<std::int8_t> data{
    0,
    20,
    50,
    100,
    -1,
    70
  };

  const auto quality =
    savo_mapping::evaluate_map_quality_placeholder(
    0.5,
    3,
    2,
    data,
    7,
    25,
    65);

  EXPECT_TRUE(quality.map_received);
  EXPECT_TRUE(quality.structurally_valid);
  EXPECT_FALSE(quality.evaluated);
  EXPECT_FALSE(quality.navigation_handoff_ready);

  EXPECT_EQ(quality.width, 3u);
  EXPECT_EQ(quality.height, 2u);
  EXPECT_DOUBLE_EQ(quality.resolution_m, 0.5);
  EXPECT_DOUBLE_EQ(quality.map_area_m2, 1.5);

  EXPECT_NEAR(quality.free_ratio, 2.0 / 6.0, 1e-9);
  EXPECT_NEAR(quality.occupied_ratio, 2.0 / 6.0, 1e-9);
  EXPECT_NEAR(quality.unknown_ratio, 1.0 / 6.0, 1e-9);

  EXPECT_DOUBLE_EQ(quality.quality_score, 0.0);
  EXPECT_EQ(quality.update_count, 7u);
  EXPECT_EQ(
    quality.reason,
    "quality_scoring_not_implemented");
}

TEST(MapQualityPlaceholderContract, RejectsCellCountMismatch)
{
  const std::vector<std::int8_t> data{0};

  const auto quality =
    savo_mapping::evaluate_map_quality_placeholder(
    0.05,
    2,
    2,
    data,
    1,
    25,
    65);

  EXPECT_FALSE(quality.structurally_valid);
  EXPECT_EQ(quality.reason, "cell_count_mismatch");
}

TEST(MapQualityPlaceholderContract, RejectsInvalidResolution)
{
  const std::vector<std::int8_t> data{0};

  const auto quality =
    savo_mapping::evaluate_map_quality_placeholder(
    0.0,
    1,
    1,
    data,
    1,
    25,
    65);

  EXPECT_FALSE(quality.structurally_valid);
  EXPECT_EQ(quality.reason, "invalid_resolution");
}

TEST(MapQualityPlaceholderContract, RejectsInvalidOccupancyValue)
{
  const std::vector<std::int8_t> data{
    static_cast<std::int8_t>(-2)
  };

  const auto quality =
    savo_mapping::evaluate_map_quality_placeholder(
    0.05,
    1,
    1,
    data,
    1,
    25,
    65);

  EXPECT_FALSE(quality.structurally_valid);
  EXPECT_EQ(quality.reason, "invalid_occupancy_value");
}

TEST(MapQualityPlaceholderContract, JsonStatesPlaceholderIsNotEvaluated)
{
  const std::vector<std::int8_t> data{0};

  const auto quality =
    savo_mapping::evaluate_map_quality_placeholder(
    0.05,
    1,
    1,
    data,
    1,
    25,
    65);

  const std::string json =
    savo_mapping::make_map_quality_json(quality);

  EXPECT_NE(
    json.find("\"structurally_valid\":true"),
    std::string::npos);

  EXPECT_NE(
    json.find("\"evaluated\":false"),
    std::string::npos);

  EXPECT_NE(
    json.find("\"navigation_handoff_ready\":false"),
    std::string::npos);

  EXPECT_NE(
    json.find("\"quality_score\":0.000000"),
    std::string::npos);
}

TEST(MonitorOnlyPolicyContract, SafePolicyPasses)
{
  const savo_mapping::MonitorOnlyPolicy policy;

  EXPECT_TRUE(
    savo_mapping::validate_monitor_only_policy(policy).empty());
}

TEST(MonitorOnlyPolicyContract, RejectsDirectMotorControl)
{
  savo_mapping::MonitorOnlyPolicy policy;
  policy.allow_direct_motor_control = true;

  EXPECT_EQ(
    savo_mapping::validate_monitor_only_policy(policy),
    "direct_motor_control_must_be_disabled");
}

TEST(MonitorOnlyPolicyContract, RejectsInternalPackageCalls)
{
  savo_mapping::MonitorOnlyPolicy policy;
  policy.allow_internal_package_calls = true;

  EXPECT_EQ(
    savo_mapping::validate_monitor_only_policy(policy),
    "internal_package_calls_must_be_disabled");
}

TEST(MonitorOnlyPolicyContract, RejectsNavigationHandoff)
{
  savo_mapping::MonitorOnlyPolicy policy;
  policy.navigation_handoff_enabled = true;

  EXPECT_EQ(
    savo_mapping::validate_monitor_only_policy(policy),
    "navigation_handoff_must_be_disabled");
}

TEST(MonitorOnlyPolicyContract, RejectsUnsupportedOptionalFeatures)
{
  savo_mapping::MonitorOnlyPolicy policy;
  policy.realsense_monitoring_enabled = true;

  EXPECT_EQ(
    savo_mapping::validate_monitor_only_policy(policy),
    "realsense_monitoring_not_implemented");

  policy.realsense_monitoring_enabled = false;
  policy.voxel_obstacle_monitoring_enabled = true;

  EXPECT_EQ(
    savo_mapping::validate_monitor_only_policy(policy),
    "voxel_obstacle_monitoring_not_implemented");

  policy.voxel_obstacle_monitoring_enabled = false;
  policy.semantic_mapping_enabled = true;

  EXPECT_EQ(
    savo_mapping::validate_monitor_only_policy(policy),
    "semantic_mapping_not_implemented");
}
