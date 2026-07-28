#include <gtest/gtest.h>
#include "savo_supervisor/supervisor_policy.hpp"

using namespace savo_supervisor;

TEST(SupervisorContract, ValidatePolicy)
{
  SupervisorPolicy policy;
  EXPECT_TRUE(policy.Validate());

  policy.publish_rate_hz = 0.0;
  EXPECT_FALSE(policy.Validate());
  EXPECT_EQ(policy.ValidationError(), "publish_rate_hz must be positive");
}
