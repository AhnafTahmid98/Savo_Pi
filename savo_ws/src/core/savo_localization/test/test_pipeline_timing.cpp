// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <sstream>

#include "gtest/gtest.h"
#include "savo_localization/pipeline_timing.hpp"

TEST(PipelineTiming, SeparatesCallbackGapWorkAndDeliveryDelay)
{
  savo_localization::CallbackTiming timing;
  timing.Start(1000000000, 4000000000, 3800000000, 3900000000);
  timing.work.Record(2000000, 500000);
  timing.Start(1732000000, 4732000000, 4700000000, 4710000000);
  timing.work.Record(1000000, 250000);
  EXPECT_EQ(timing.count, 2U);
  EXPECT_EQ(timing.last_start_ns, 1732000000);
  EXPECT_DOUBLE_EQ(timing.gap_s, 0.732);
  EXPECT_DOUBLE_EQ(timing.max_gap_s, 0.732);
  EXPECT_DOUBLE_EQ(timing.work.last_wall_s, 0.001);
  EXPECT_DOUBLE_EQ(timing.work.max_wall_s, 0.002);
  EXPECT_DOUBLE_EQ(timing.work.last_cpu_s, 0.00025);
  EXPECT_DOUBLE_EQ(timing.work.max_cpu_s, 0.0005);
  EXPECT_DOUBLE_EQ(timing.source_to_callback_s, 0.032);
  EXPECT_DOUBLE_EQ(timing.receive_to_callback_s, 0.022);
  EXPECT_DOUBLE_EQ(timing.max_source_to_callback_s, 0.2);
  EXPECT_DOUBLE_EQ(timing.max_receive_to_callback_s, 0.1);
  timing.RecordHeader(5000000000, 4800000000);
  EXPECT_EQ(timing.header_stamp_ns, 4800000000);
  EXPECT_DOUBLE_EQ(timing.header_age_at_callback_s, 0.2);
}

TEST(PipelineTiming, MissingMetadataIsUnavailableAndClockSkewStaysVisible)
{
  savo_localization::CallbackTiming timing;
  timing.Start(100, 200, 0, 0);
  std::ostringstream missing;
  timing.WriteJson(missing);
  EXPECT_NE(missing.str().find("\"source_to_callback_s\":null"), std::string::npos);
  EXPECT_NE(missing.str().find("\"receive_to_callback_s\":null"), std::string::npos);
  EXPECT_EQ(timing.gap_s, -1.0);
  EXPECT_NE(missing.str().find("\"max_source_to_callback_s\":null"), std::string::npos);
  EXPECT_NE(missing.str().find("\"max_receive_to_callback_s\":null"), std::string::npos);
  EXPECT_NE(missing.str().find("\"max_cpu_s\":null"), std::string::npos);
  timing.Start(200, 3000000000, 4000000000, 4000000000);
  EXPECT_DOUBLE_EQ(timing.source_to_callback_s, -1.0);
  EXPECT_DOUBLE_EQ(timing.receive_to_callback_s, -1.0);
  EXPECT_DOUBLE_EQ(timing.max_source_to_callback_s, -1.0);
  EXPECT_DOUBLE_EQ(timing.max_receive_to_callback_s, -1.0);
  timing.work.Record(100, -1);
  std::ostringstream skewed;
  timing.WriteJson(skewed);
  EXPECT_NE(skewed.str().find("\"cpu_s\":null"), std::string::npos);
}
