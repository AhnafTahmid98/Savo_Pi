#include <gtest/gtest.h>
#include "savo_supervisor/supervisor_policy.hpp"

using namespace savo_supervisor;

TEST(SupervisorPolicy, DefaultLocalizationConfig)
{
  const auto config = SupervisorPolicy::DefaultLocalizationConfig();

  EXPECT_EQ(config.name, "localization");
  EXPECT_TRUE(config.enabled);
  EXPECT_TRUE(config.required);
  EXPECT_EQ(config.health_topic, "/savo_localization/health");
  EXPECT_EQ(config.summary_topic, "/savo_localization/state_summary");
  EXPECT_EQ(config.heartbeat_topic, "/savo_localization/heartbeat");
  EXPECT_EQ(config.expected_schema_version, 1);
}
