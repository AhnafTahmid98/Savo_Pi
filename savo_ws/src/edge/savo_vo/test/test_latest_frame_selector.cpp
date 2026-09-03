#include "savo_vo/latest_frame_selector.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{

using savo_vo::LatestFrameSelector;

TEST(LatestFrameSelectorTest, KeepsOnlyNewestPendingFrame)
{
  LatestFrameSelector selector;
  for (int index = 1; index <= 1000; ++index) {
    EXPECT_TRUE(selector.offer(static_cast<double>(index) * 0.01));
  }

  EXPECT_EQ(selector.pending_count(), 1U);
  const auto selected = selector.take();
  ASSERT_TRUE(selected.has_value());
  EXPECT_DOUBLE_EQ(selected.value(), 10.0);
  EXPECT_EQ(selector.pending_count(), 0U);
}

TEST(LatestFrameSelectorTest, LatestSynchronizedFrameWins)
{
  LatestFrameSelector selector;
  ASSERT_TRUE(selector.offer(12.10));
  ASSERT_TRUE(selector.offer(12.20));

  const auto selected = selector.take();
  ASSERT_TRUE(selected.has_value());
  EXPECT_DOUBLE_EQ(selected.value(), 12.20);
  EXPECT_EQ(selector.pending_count(), 0U);
}

TEST(LatestFrameSelectorTest, RejectsObsoleteAndNonFiniteFrames)
{
  LatestFrameSelector selector;
  ASSERT_TRUE(selector.offer(5.0));
  EXPECT_FALSE(selector.offer(5.0));
  EXPECT_FALSE(selector.offer(4.9));
  EXPECT_FALSE(selector.offer(std::numeric_limits<double>::infinity()));

  EXPECT_DOUBLE_EQ(selector.take().value_or(0.0), 5.0);
}

TEST(LatestFrameSelectorTest, NormalLowerCadenceCreatesNoBacklog)
{
  LatestFrameSelector selector;
  for (int index = 1; index <= 20; ++index) {
    const double stamp_s = static_cast<double>(index) * 0.1;
    ASSERT_TRUE(selector.offer(stamp_s));
    EXPECT_DOUBLE_EQ(selector.take().value_or(0.0), stamp_s);
    EXPECT_EQ(selector.pending_count(), 0U);
  }
}

TEST(FrameIntervalTest, RetainsInvalidIntervalProtection)
{
  EXPECT_TRUE(savo_vo::valid_frame_interval(1.0, 1.1, 0.20));
  EXPECT_FALSE(savo_vo::valid_frame_interval(1.0, 1.0, 0.20));
  EXPECT_FALSE(savo_vo::valid_frame_interval(1.0, 0.9, 0.20));
  EXPECT_FALSE(savo_vo::valid_frame_interval(1.0, 1.201, 0.20));
}

}  // namespace
