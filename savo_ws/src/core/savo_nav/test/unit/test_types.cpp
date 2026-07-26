// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "savo_nav/types.hpp"

namespace
{

TEST(ValidationTypesTest, DefaultResultIsValid)
{
  const savo_nav::ValidationResult result;

  EXPECT_TRUE(result.IsValid());
  EXPECT_EQ(result.reason, "valid");
}

TEST(ValidationTypesTest, ConvertsEveryValidationCode)
{
  using Code = savo_nav::ValidationCode;

  const std::vector<
    std::pair<Code, std::string_view>> expected{
    {Code::kValid, "valid"},
    {Code::kEmptyIdentifier, "empty_identifier"},
    {Code::kInvalidIdentifier, "invalid_identifier"},
    {Code::kInvalidFrame, "invalid_frame"},
    {Code::kInvalidState, "invalid_state"},
    {Code::kInvalidCombination, "invalid_combination"},
    {Code::kMissingMapId, "missing_map_id"},
    {Code::kInvalidAuthority, "invalid_authority"},
    {Code::kInvalidResult, "invalid_result"},
    {Code::kNotReady, "not_ready"},
    {Code::kInvalidPose, "invalid_pose"},
    {
      Code::kCoordinateOutOfBounds,
      "coordinate_out_of_bounds"
    },
    {Code::kMapUnavailable, "map_unavailable"},
    {Code::kMapMismatch, "map_mismatch"},
    {
      Code::kCancellationRequested,
      "cancellation_requested"
    }
  };

  for (const auto & item : expected) {
    EXPECT_EQ(
      savo_nav::ToString(item.first),
      item.second);
  }
}

TEST(ValidationTypesTest, FailureResultIsNotValid)
{
  const savo_nav::ValidationResult result{
    savo_nav::ValidationCode::kInvalidFrame,
    "invalid_frame"
  };

  EXPECT_FALSE(result.IsValid());
}

}  // namespace
