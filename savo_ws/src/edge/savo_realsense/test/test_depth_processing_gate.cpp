// Copyright 2026 Ahnaf Tahmid

#include "savo_realsense/depth_processing_gate.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace
{

using savo_realsense::DepthProcessingGate;

TEST(DepthProcessingGateTest, SelectsFreshFramesAtBoundedCadence)
{
  DepthProcessingGate gate(15.0);
  EXPECT_TRUE(gate.should_process(1.000));
  EXPECT_FALSE(gate.should_process(1.033));
  EXPECT_FALSE(gate.should_process(1.066));
  EXPECT_TRUE(gate.should_process(1.067));
}

TEST(DepthProcessingGateTest, NeverBuildsAQueue)
{
  DepthProcessingGate gate(10.0);
  EXPECT_TRUE(gate.should_process(5.0));
  for (int index = 1; index < 10; ++index) {
    EXPECT_FALSE(gate.should_process(5.0 + 0.01 * index));
  }
  EXPECT_TRUE(gate.should_process(5.101));
}

TEST(DepthProcessingGateTest, RecoversFromClockRollback)
{
  DepthProcessingGate gate(10.0);
  EXPECT_TRUE(gate.should_process(10.0));
  EXPECT_TRUE(gate.should_process(9.0));
  EXPECT_FALSE(gate.should_process(9.05));
}

TEST(DepthProcessingGateTest, RejectsInvalidRateAndTime)
{
  EXPECT_THROW(DepthProcessingGate(0.0), std::invalid_argument);
  EXPECT_THROW(
    DepthProcessingGate(std::numeric_limits<double>::infinity()),
    std::invalid_argument);

  DepthProcessingGate gate(10.0);
  EXPECT_FALSE(
    gate.should_process(std::numeric_limits<double>::quiet_NaN()));
}

}  // namespace
