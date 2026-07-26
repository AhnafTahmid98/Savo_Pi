#include "savo_mapping/scan360_quality.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace
{

namespace scan360 =
  savo_mapping::scan360;

scan360::Scan360QualityInput
make_quality_input(
  std::size_t target_count,
  double yaw_error_rad = 0.02)
{
  scan360::Scan360QualityInput input;

  input.planned_target_count =
    target_count;

  for (
    std::size_t index = 0;
    index < target_count;
    ++index)
  {
    scan360::Scan360Observation
      observation;

    observation.target_index = index;
    observation.yaw_error_rad =
      yaw_error_rad;

    observation.valid_scan_points = 100;
    observation.settled = true;
    observation.map_updated = true;

    input.observations.push_back(
      observation);
  }

  return input;
}

}  // namespace

TEST(
  Scan360QualityContract,
  GradeStringsAreStable)
{
  EXPECT_EQ(
    std::string{
    scan360::to_string(
        scan360::QualityGrade::
      InsufficientData)},
    "insufficient_data");

  EXPECT_EQ(
    std::string{
    scan360::to_string(
        scan360::QualityGrade::
      Rejected)},
    "rejected");

  EXPECT_EQ(
    std::string{
    scan360::to_string(
        scan360::QualityGrade::
      Acceptable)},
    "acceptable");

  EXPECT_EQ(
    std::string{
    scan360::to_string(
        scan360::QualityGrade::Good)},
    "good");
}

TEST(
  Scan360QualityContract,
  DefaultThresholdsAreValid)
{
  EXPECT_TRUE(
    scan360::thresholds_are_valid(
      scan360::
      Scan360QualityThresholds{}));
}

TEST(
  Scan360QualityContract,
  CompleteAccurateScanIsGood)
{
  const auto result =
    scan360::evaluate_quality(
    make_quality_input(12U),
    scan360::
    Scan360QualityThresholds{});

  EXPECT_TRUE(result.evaluated);
  EXPECT_TRUE(result.accepted);

  EXPECT_EQ(
    result.grade,
    scan360::QualityGrade::Good);

  EXPECT_EQ(
    result.reason,
    "quality_good");

  EXPECT_DOUBLE_EQ(
    result.completion_ratio,
    1.0);

  EXPECT_DOUBLE_EQ(
    result.settled_ratio,
    1.0);

  EXPECT_DOUBLE_EQ(
    result.map_update_ratio,
    1.0);

  EXPECT_DOUBLE_EQ(
    result.scan_sufficient_ratio,
    1.0);
}

TEST(
  Scan360QualityContract,
  PassingButImperfectScanIsAcceptable)
{
  auto input =
    make_quality_input(
    10U,
    0.06);

  input.observations.pop_back();

  const auto result =
    scan360::evaluate_quality(
    input,
    scan360::
    Scan360QualityThresholds{});

  EXPECT_TRUE(result.evaluated);
  EXPECT_TRUE(result.accepted);

  EXPECT_EQ(
    result.grade,
    scan360::QualityGrade::
    Acceptable);

  EXPECT_EQ(
    result.reason,
    "quality_acceptable");

  EXPECT_DOUBLE_EQ(
    result.completion_ratio,
    0.9);
}

TEST(
  Scan360QualityContract,
  IncompleteScanIsRejected)
{
  auto input =
    make_quality_input(10U);

  input.observations.resize(8U);

  const auto result =
    scan360::evaluate_quality(
    input,
    scan360::
    Scan360QualityThresholds{});

  EXPECT_TRUE(result.evaluated);
  EXPECT_FALSE(result.accepted);

  EXPECT_EQ(
    result.grade,
    scan360::QualityGrade::Rejected);

  EXPECT_EQ(
    result.reason,
    "completion_ratio_below_threshold");
}

TEST(
  Scan360QualityContract,
  UnsettledTargetsAreRejected)
{
  auto input =
    make_quality_input(10U);

  input.observations[8].settled = false;
  input.observations[9].settled = false;

  const auto result =
    scan360::evaluate_quality(
    input,
    scan360::
    Scan360QualityThresholds{});

  EXPECT_FALSE(result.accepted);

  EXPECT_EQ(
    result.reason,
    "settled_ratio_below_threshold");
}

TEST(
  Scan360QualityContract,
  MissingMapUpdatesAreRejected)
{
  auto input =
    make_quality_input(8U);

  input.observations[5].map_updated = false;
  input.observations[6].map_updated = false;
  input.observations[7].map_updated = false;

  const auto result =
    scan360::evaluate_quality(
    input,
    scan360::
    Scan360QualityThresholds{});

  EXPECT_FALSE(result.accepted);

  EXPECT_EQ(
    result.reason,
    "map_update_ratio_below_threshold");
}

TEST(
  Scan360QualityContract,
  InsufficientScanPointsAreRejected)
{
  auto input =
    make_quality_input(10U);

  input.observations[8].valid_scan_points = 5U;
  input.observations[9].valid_scan_points = 5U;

  const auto result =
    scan360::evaluate_quality(
    input,
    scan360::
    Scan360QualityThresholds{});

  EXPECT_FALSE(result.accepted);

  EXPECT_EQ(
    result.reason,
    "scan_point_ratio_below_threshold");
}

TEST(
  Scan360QualityContract,
  MeanYawErrorIsRejected)
{
  const auto result =
    scan360::evaluate_quality(
    make_quality_input(
      8U,
      0.10),
    scan360::
    Scan360QualityThresholds{});

  EXPECT_FALSE(result.accepted);

  EXPECT_EQ(
    result.reason,
    "mean_yaw_error_above_threshold");
}

TEST(
  Scan360QualityContract,
  PeakYawErrorIsRejected)
{
  auto input =
    make_quality_input(
    10U,
    0.02);

  input.observations[3].
  yaw_error_rad = 0.20;

  const auto result =
    scan360::evaluate_quality(
    input,
    scan360::
    Scan360QualityThresholds{});

  EXPECT_FALSE(result.accepted);

  EXPECT_EQ(
    result.reason,
    "peak_yaw_error_above_threshold");
}

TEST(
  Scan360QualityContract,
  DuplicateObservationIsInvalid)
{
  auto input =
    make_quality_input(4U);

  input.observations.push_back(
    input.observations.front());

  const auto result =
    scan360::evaluate_quality(
    input,
    scan360::
    Scan360QualityThresholds{});

  EXPECT_FALSE(result.evaluated);
  EXPECT_FALSE(result.accepted);

  EXPECT_EQ(
    result.reason,
    "duplicate_target_observation");
}

TEST(
  Scan360QualityContract,
  OutOfRangeTargetIsInvalid)
{
  auto input =
    make_quality_input(4U);

  input.observations.front().
  target_index = 4U;

  const auto result =
    scan360::evaluate_quality(
    input,
    scan360::
    Scan360QualityThresholds{});

  EXPECT_FALSE(result.evaluated);

  EXPECT_EQ(
    result.reason,
    "target_index_out_of_range");
}

TEST(
  Scan360QualityContract,
  NonFiniteYawErrorIsInvalid)
{
  auto input =
    make_quality_input(4U);

  input.observations.front().
  yaw_error_rad = std::nan("");

  const auto result =
    scan360::evaluate_quality(
    input,
    scan360::
    Scan360QualityThresholds{});

  EXPECT_FALSE(result.evaluated);

  EXPECT_EQ(
    result.reason,
    "yaw_error_not_finite");
}

TEST(
  Scan360QualityContract,
  InvalidThresholdsAreRejected)
{
  auto thresholds =
    scan360::
    Scan360QualityThresholds{};

  thresholds.minimum_completion_ratio =
    1.1;

  EXPECT_FALSE(
    scan360::thresholds_are_valid(
      thresholds));

  const auto result =
    scan360::evaluate_quality(
    make_quality_input(4U),
    thresholds);

  EXPECT_FALSE(result.evaluated);

  EXPECT_EQ(
    result.reason,
    "invalid_thresholds");
}
