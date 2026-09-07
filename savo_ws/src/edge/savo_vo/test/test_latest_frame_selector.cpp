#include "savo_vo/latest_frame_selector.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

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

TEST(FrameIntervalTest, CameraRatesFrom15To25HzRemainValidAt12HzProcessing)
{
  constexpr double processing_rate_hz = 12.0;
  constexpr double max_interval_s = 0.20;

  for (const double source_rate_hz : {15.0, 20.0, 25.0}) {
    LatestFrameSelector selector;
    double next_source_stamp_s = 0.0;
    double previous_processed_stamp_s = 0.0;
    bool have_previous = false;
    std::size_t processed_count = 0U;

    for (int tick = 0; tick <= 24; ++tick) {
      const double processing_time_s =
        static_cast<double>(tick) / processing_rate_hz;
      while (next_source_stamp_s <= processing_time_s + 1e-9) {
        ASSERT_TRUE(selector.offer(next_source_stamp_s));
        next_source_stamp_s += 1.0 / source_rate_hz;
      }

      const auto selected = selector.take();
      if (!selected.has_value()) {
        continue;
      }
      EXPECT_EQ(selector.pending_count(), 0U);
      if (have_previous) {
        EXPECT_TRUE(savo_vo::valid_frame_interval(
          previous_processed_stamp_s,
          selected.value(),
          max_interval_s));
      }
      previous_processed_stamp_s = selected.value();
      have_previous = true;
      ++processed_count;
    }

    EXPECT_GE(processed_count, 20U);
  }
}

TEST(FrameIntervalTest, LongSourceGapReseedsOnceThenNextFrameRecovers)
{
  double reference_stamp_s = 1.0;

  const double post_gap_stamp_s = 1.4;
  EXPECT_FALSE(savo_vo::valid_frame_interval(
    reference_stamp_s, post_gap_stamp_s, 0.20));

  // The node replaces the reference without estimating or publishing motion.
  reference_stamp_s = post_gap_stamp_s;
  const double recovered_stamp_s = post_gap_stamp_s + (1.0 / 15.0);
  EXPECT_TRUE(savo_vo::valid_frame_interval(
    reference_stamp_s, recovered_stamp_s, 0.20));
}

}  // namespace
