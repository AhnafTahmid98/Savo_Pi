#include <gtest/gtest.h>

#include <limits>

#include "savo_locations/model.hpp"
#include "savo_locations/validation.hpp"


namespace
{

savo_locations::LocationDraft
make_valid_location()
{
  savo_locations::LocationDraft location;

  location.location_id = "A201";
  location.display_name = "Room A201";
  location.aliases = {
    "A 201",
    "Classroom A201",
  };

  location.semantic_type = "classroom";

  location.map.map_id = "campus_main";
  location.map.map_revision = 7U;
  location.map.map_release_id =
    "campus_main_release_2026_07";

  location.approach_pose.frame_id = "map";
  location.approach_pose.x = 12.4;
  location.approach_pose.y = 7.8;
  location.approach_pose.qw = 1.0;

  savo_locations::PoseData tag_pose;
  tag_pose.frame_id = "map";
  tag_pose.x = 12.8;
  tag_pose.y = 8.1;
  tag_pose.qw = 1.0;

  location.tag_pose_map = tag_pose;

  location.tag.family = "tag36h11";
  location.tag.id = 27;

  return location;
}


savo_locations::CandidateDraft
make_valid_candidate()
{
  savo_locations::CandidateDraft candidate;

  candidate.candidate_id =
    "candidate-campus-main-27";

  candidate.map.map_id = "campus_main";
  candidate.map.map_revision = 7U;

  candidate.tag.family = "tag36h11";
  candidate.tag.id = 27;

  candidate.tag_pose_map.frame_id = "map";
  candidate.tag_pose_map.x = 12.8;
  candidate.tag_pose_map.y = 8.1;
  candidate.tag_pose_map.qw = 1.0;

  candidate.detection_quality = 0.95;
  candidate.accepted_observations = 8U;
  candidate.position_stddev_m = 0.02;
  candidate.yaw_stddev_rad = 0.03;

  candidate.suggested_location_id = "A201";
  candidate.suggested_display_name =
    "Room A201";

  candidate.suggested_aliases = {
    "A 201",
  };

  candidate.suggested_semantic_type =
    "classroom";

  return candidate;
}

}  // namespace


TEST(LocationValidation, AcceptsValidLocation)
{
  const auto result =
    savo_locations::validate_location_draft(
      make_valid_location());

  EXPECT_TRUE(result.valid());
  EXPECT_TRUE(result.issues().empty());
}


TEST(LocationValidation, RejectsNonCanonicalId)
{
  auto location = make_valid_location();
  location.location_id = "a 201";

  const auto result =
    savo_locations::validate_location_draft(
      location);

  EXPECT_FALSE(result.valid());

  EXPECT_TRUE(
    result.has(
      savo_locations::ValidationCode::
        kInvalidFormat));
}


TEST(LocationValidation, RejectsZeroMapRevision)
{
  auto location = make_valid_location();
  location.map.map_revision = 0U;

  const auto result =
    savo_locations::validate_location_draft(
      location);

  EXPECT_TRUE(
    result.has(
      savo_locations::ValidationCode::
        kMapRevisionZero));
}


TEST(LocationValidation, RejectsWrongPoseFrame)
{
  auto location = make_valid_location();

  location.approach_pose.frame_id =
    "base_link";

  const auto result =
    savo_locations::validate_location_draft(
      location);

  EXPECT_TRUE(
    result.has(
      savo_locations::ValidationCode::
        kWrongFrame));
}


TEST(LocationValidation, RejectsInvalidQuaternion)
{
  auto location = make_valid_location();

  location.approach_pose.qx = 0.0;
  location.approach_pose.qy = 0.0;
  location.approach_pose.qz = 0.0;
  location.approach_pose.qw = 0.0;

  const auto result =
    savo_locations::validate_location_draft(
      location);

  EXPECT_TRUE(
    result.has(
      savo_locations::ValidationCode::
        kInvalidQuaternion));
}


TEST(LocationValidation, RejectsDuplicateAliasKeys)
{
  auto location = make_valid_location();

  location.aliases.push_back(
    "room-a201");

  const auto result =
    savo_locations::validate_location_draft(
      location);

  EXPECT_TRUE(
    result.has(
      savo_locations::ValidationCode::
        kDuplicateNormalizedKey));
}


TEST(LocationValidation, RejectsNonFinitePose)
{
  auto location = make_valid_location();

  location.approach_pose.x =
    std::numeric_limits<double>::
      quiet_NaN();

  const auto result =
    savo_locations::validate_location_draft(
      location);

  EXPECT_TRUE(
    result.has(
      savo_locations::ValidationCode::
        kNonFiniteNumber));
}


TEST(LocationValidation, AcceptsValidCandidate)
{
  const auto result =
    savo_locations::validate_candidate_draft(
      make_valid_candidate());

  EXPECT_TRUE(result.valid());
}


TEST(LocationValidation, RejectsInvalidCandidateQuality)
{
  auto candidate = make_valid_candidate();

  candidate.detection_quality = 1.5;
  candidate.accepted_observations = 0U;

  const auto result =
    savo_locations::validate_candidate_draft(
      candidate);

  EXPECT_FALSE(result.valid());

  EXPECT_TRUE(
    result.has(
      savo_locations::ValidationCode::
        kOutOfRange));
}


TEST(LocationValidation, ReasonStringsAreStable)
{
  using savo_locations::ValidationCode;
  using savo_locations::to_string;

  EXPECT_EQ(
    to_string(
      ValidationCode::
        kDuplicateNormalizedKey),
    "duplicate_normalized_key");

  EXPECT_EQ(
    to_string(
      ValidationCode::
        kInvalidQuaternion),
    "invalid_quaternion");
}
