#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "savo_locations/location_catalog.hpp"


namespace
{

savo_locations::PoseData make_pose(
  const double x,
  const double y)
{
  savo_locations::PoseData pose;
  pose.frame_id = "map";
  pose.x = x;
  pose.y = y;
  pose.qw = 1.0;
  return pose;
}


savo_locations::CandidateDraft make_candidate(
  const std::string & candidate_id,
  const std::string & map_id,
  const std::uint32_t map_revision,
  const std::int32_t tag_id,
  const std::string & suggested_location_id)
{
  savo_locations::CandidateDraft candidate;

  candidate.candidate_id = candidate_id;

  candidate.map.map_id = map_id;
  candidate.map.map_revision = map_revision;

  candidate.tag.family = "tag36h11";
  candidate.tag.id = tag_id;

  candidate.tag_pose_map =
    make_pose(12.8, 8.1);

  candidate.detection_quality = 0.95;
  candidate.accepted_observations = 8U;
  candidate.position_stddev_m = 0.02;
  candidate.yaw_stddev_rad = 0.03;

  candidate.suggested_location_id =
    suggested_location_id;

  candidate.suggested_display_name =
    "Room " + suggested_location_id;

  candidate.suggested_aliases = {
    suggested_location_id.substr(0U, 1U) +
    " " +
    suggested_location_id.substr(1U),
  };

  candidate.suggested_semantic_type =
    "classroom";

  return candidate;
}


savo_locations::LocationRecordData make_location(
  const std::string & location_id,
  const std::string & map_id,
  const std::uint32_t map_revision,
  const std::int32_t tag_id,
  const std::vector<std::string> & aliases = {})
{
  savo_locations::LocationRecordData record;

  record.state =
    savo_locations::LocationState::kApproved;

  record.enabled = true;
  record.record_revision = 1U;

  record.location.location_id =
    location_id;

  record.location.display_name =
    "Room " + location_id;

  record.location.aliases = aliases;
  record.location.semantic_type = "classroom";

  record.location.map.map_id = map_id;

  record.location.map.map_revision =
    map_revision;

  record.location.approach_pose =
    make_pose(10.0, 5.0);

  record.location.tag.family = "tag36h11";
  record.location.tag.id = tag_id;

  return record;
}


savo_locations::ApprovalRequest
make_approval(
  const std::string & candidate_id,
  const std::uint64_t revision)
{
  savo_locations::ApprovalRequest request;

  request.candidate_id = candidate_id;

  request.expected_candidate_revision =
    revision;

  request.approach_pose =
    make_pose(12.0, 7.5);

  return request;
}

}  // namespace


TEST(LocationCatalog, RegistersCandidate)
{
  savo_locations::InMemoryLocationCatalog catalog;

  const auto result =
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        27,
        "A201"));

  ASSERT_TRUE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::CandidateMutationCode::
      kRegistered);

  ASSERT_TRUE(result.candidate.has_value());

  EXPECT_EQ(
    result.candidate->state,
    savo_locations::CandidateState::
      kPendingReview);

  EXPECT_EQ(
    result.candidate->candidate_revision,
    1U);

  EXPECT_EQ(catalog.candidate_size(), 1U);
}


TEST(LocationCatalog, RejectsDuplicateCandidateId)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        27,
        "A201")).success);

  const auto duplicate =
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        28,
        "A202"));

  EXPECT_FALSE(duplicate.success);

  EXPECT_EQ(
    duplicate.code,
    savo_locations::CandidateMutationCode::
      kCandidateIdConflict);
}


TEST(LocationCatalog, RejectsPendingTagConflict)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-27-a",
        "campus_main",
        7U,
        27,
        "A201")).success);

  const auto conflict =
    catalog.register_candidate(
      make_candidate(
        "candidate-27-b",
        "campus_main",
        7U,
        27,
        "A202"));

  EXPECT_FALSE(conflict.success);

  EXPECT_EQ(
    conflict.code,
    savo_locations::CandidateMutationCode::
      kTagConflict);
}


TEST(LocationCatalog, AllowsSameTagAcrossMaps)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "north-candidate",
        "campus_north",
        3U,
        27,
        "N201")).success);

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "south-candidate",
        "campus_south",
        4U,
        27,
        "S201")).success);

  EXPECT_EQ(catalog.candidate_size(), 2U);
}


TEST(LocationCatalog, RejectsTagUsedByLocation)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.insert_location(
      make_location(
        "A201",
        "campus_main",
        7U,
        27)).success);

  const auto candidate =
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        27,
        "A202"));

  EXPECT_FALSE(candidate.success);

  EXPECT_EQ(
    candidate.code,
    savo_locations::CandidateMutationCode::
      kTagConflict);
}


TEST(LocationCatalog, ReplacesPendingCandidate)
{
  savo_locations::InMemoryLocationCatalog catalog;

  auto candidate =
    make_candidate(
      "candidate-27",
      "campus_main",
      7U,
      27,
      "A201");

  ASSERT_TRUE(
    catalog.register_candidate(
      candidate).success);

  candidate.detection_quality = 0.99;
  candidate.accepted_observations = 12U;

  const auto updated =
    catalog.replace_candidate(
      "candidate-27",
      candidate,
      1U);

  ASSERT_TRUE(updated.success);
  ASSERT_TRUE(updated.candidate.has_value());

  EXPECT_EQ(
    updated.candidate->candidate_revision,
    2U);

  EXPECT_DOUBLE_EQ(
    updated.candidate
      ->candidate
      .detection_quality,
    0.99);
}


TEST(LocationCatalog, RejectsStaleCandidateUpdate)
{
  savo_locations::InMemoryLocationCatalog catalog;

  auto candidate =
    make_candidate(
      "candidate-27",
      "campus_main",
      7U,
      27,
      "A201");

  ASSERT_TRUE(
    catalog.register_candidate(
      candidate).success);

  const auto stale =
    catalog.replace_candidate(
      "candidate-27",
      candidate,
      99U);

  EXPECT_FALSE(stale.success);

  EXPECT_EQ(
    stale.code,
    savo_locations::CandidateMutationCode::
      kStaleRevision);
}


TEST(LocationCatalog, RejectsCandidateWithReason)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        27,
        "A201")).success);

  const auto rejected =
    catalog.reject_candidate(
      "candidate-27",
      1U,
      "  approach pose is blocked  ");

  ASSERT_TRUE(rejected.success);
  ASSERT_TRUE(rejected.candidate.has_value());

  EXPECT_EQ(
    rejected.candidate->state,
    savo_locations::CandidateState::
      kRejected);

  EXPECT_EQ(
    rejected.candidate->candidate_revision,
    2U);

  EXPECT_EQ(
    rejected.candidate->review_reason,
    "approach pose is blocked");
}


TEST(LocationCatalog, RequiresRejectionReason)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        27,
        "A201")).success);

  const auto rejected =
    catalog.reject_candidate(
      "candidate-27",
      1U,
      "   ");

  EXPECT_FALSE(rejected.success);

  EXPECT_EQ(
    rejected.code,
    savo_locations::CandidateMutationCode::
      kEmptyReviewReason);
}


TEST(LocationCatalog, CannotApproveRejectedCandidate)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        27,
        "A201")).success);

  ASSERT_TRUE(
    catalog.reject_candidate(
      "candidate-27",
      1U,
      "invalid landmark").success);

  const auto approval =
    catalog.approve_candidate(
      make_approval(
        "candidate-27",
        2U));

  EXPECT_FALSE(approval.success);

  EXPECT_EQ(
    approval.code,
    savo_locations::ApprovalCode::
      kCandidateNotPending);
}


TEST(LocationCatalog, ApprovesCandidateAtomically)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        27,
        "A201")).success);

  const auto approval =
    catalog.approve_candidate(
      make_approval(
        "candidate-27",
        1U));

  ASSERT_TRUE(approval.success);
  ASSERT_TRUE(approval.candidate.has_value());
  ASSERT_TRUE(approval.location.has_value());

  EXPECT_EQ(
    approval.code,
    savo_locations::ApprovalCode::
      kApproved);

  EXPECT_EQ(
    approval.candidate->state,
    savo_locations::CandidateState::
      kApproved);

  EXPECT_EQ(
    approval.candidate->candidate_revision,
    2U);

  EXPECT_EQ(
    approval.candidate
      ->approved_location_id,
    "A201");

  EXPECT_EQ(
    approval.location
      ->source_candidate_id,
    "candidate-27");

  EXPECT_EQ(catalog.location_size(), 1U);
  EXPECT_EQ(catalog.candidate_size(), 1U);

  const auto resolved =
    catalog.resolve_location("A201");

  ASSERT_TRUE(resolved.resolved);
}


TEST(LocationCatalog, CandidateOwnsMapAndTagOnApproval)
{
  savo_locations::InMemoryLocationCatalog catalog;

  auto candidate =
    make_candidate(
      "candidate-27",
      "campus_main",
      7U,
      27,
      "A201");

  candidate.tag_pose_map =
    make_pose(12.8, 8.1);

  ASSERT_TRUE(
    catalog.register_candidate(
      candidate).success);

  auto request =
    make_approval(
      "candidate-27",
      1U);

  request.location_id = "a 201";
  request.display_name = "Room A201";

  const auto approval =
    catalog.approve_candidate(request);

  ASSERT_TRUE(approval.success);
  ASSERT_TRUE(approval.location.has_value());

  EXPECT_EQ(
    approval.location
      ->location
      .location_id,
    "A_201");

          // The inherited suggestion "A 201" becomes
          // redundant when the final ID becomes A_201.
          EXPECT_TRUE(
            approval.location
              ->location
              .aliases
              .empty());

  EXPECT_EQ(
    approval.location
      ->location
      .map
      .map_id,
    "campus_main");

  EXPECT_EQ(
    approval.location
      ->location
      .map
      .map_revision,
    7U);

  EXPECT_EQ(
    approval.location
      ->location
      .tag
      .id,
    27);

  ASSERT_TRUE(
    approval.location
      ->location
      .tag_pose_map
      .has_value());

  EXPECT_DOUBLE_EQ(
    approval.location
      ->location
      .tag_pose_map
      ->x,
    12.8);
}


        TEST(
          LocationCatalog,
          ExplicitRedundantAliasRemainsInvalid)
        {
          savo_locations::InMemoryLocationCatalog catalog;

          ASSERT_TRUE(
            catalog.register_candidate(
              make_candidate(
                "candidate-27",
                "campus_main",
                7U,
                27,
                "A201")).success);

          auto request =
            make_approval(
              "candidate-27",
              1U);

          request.location_id = "a 201";

          // Explicit operator aliases remain strict.
          request.aliases = {
            "A 201",
          };

          const auto result =
            catalog.approve_candidate(request);

          EXPECT_FALSE(result.success);

          EXPECT_EQ(
            result.code,
            savo_locations::ApprovalCode::
              kInvalidLocation);

          const auto candidate =
            catalog.get_candidate(
              "candidate-27");

          ASSERT_TRUE(candidate.has_value());

          EXPECT_EQ(
            candidate->state,
            savo_locations::CandidateState::
              kPendingReview);

          EXPECT_EQ(
            candidate->candidate_revision,
            1U);
        }


TEST(LocationCatalog, ApprovalRequiresApproachPose)
{
  savo_locations::InMemoryLocationCatalog catalog;

  auto candidate =
    make_candidate(
      "candidate-27",
      "campus_main",
      7U,
      27,
      "A201");

  candidate.approach_pose.reset();

  ASSERT_TRUE(
    catalog.register_candidate(
      candidate).success);

  savo_locations::ApprovalRequest request;
  request.candidate_id = "candidate-27";
  request.expected_candidate_revision = 1U;

  const auto result =
    catalog.approve_candidate(request);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::ApprovalCode::
      kMissingApproachPose);
}


TEST(LocationCatalog, ApprovalRejectsStaleRevision)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        27,
        "A201")).success);

  const auto result =
    catalog.approve_candidate(
      make_approval(
        "candidate-27",
        99U));

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::ApprovalCode::
      kStaleRevision);
}


TEST(LocationCatalog, FailedApprovalLeavesCandidatePending)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.insert_location(
      make_location(
        "A201",
        "campus_main",
        7U,
        10,
        {"Student service"})).success);

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-27",
        "campus_main",
        7U,
        27,
        "B201")).success);

  auto request =
    make_approval(
      "candidate-27",
      1U);

  request.location_id = "B201";
  request.display_name = "Room B201";

  request.aliases = {
    "student-service",
  };

  const auto result =
    catalog.approve_candidate(request);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::ApprovalCode::
      kLocationConflict);

  EXPECT_EQ(catalog.location_size(), 1U);

  const auto candidate =
    catalog.get_candidate(
      "candidate-27");

  ASSERT_TRUE(candidate.has_value());

  EXPECT_EQ(
    candidate->state,
    savo_locations::CandidateState::
      kPendingReview);

  EXPECT_EQ(
    candidate->candidate_revision,
    1U);

  EXPECT_TRUE(
    candidate->approved_location_id.empty());
}


TEST(LocationCatalog, CandidateListIsDeterministic)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-c",
        "campus_main",
        7U,
        31,
        "C301")).success);

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-a",
        "campus_main",
        7U,
        27,
        "A201")).success);

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-b",
        "campus_main",
        7U,
        28,
        "B101")).success);

  const auto candidates =
    catalog.list_candidates();

  ASSERT_EQ(candidates.size(), 3U);

  EXPECT_EQ(
    candidates[0].candidate.candidate_id,
    "candidate-a");

  EXPECT_EQ(
    candidates[1].candidate.candidate_id,
    "candidate-b");

  EXPECT_EQ(
    candidates[2].candidate.candidate_id,
    "candidate-c");
}


TEST(LocationCatalog, RestoresPersistedCandidateStates)
{
  savo_locations::InMemoryLocationCatalog catalog;

  auto pending = make_candidate(
    "candidate-pending",
    "campus_main",
    7U,
    27,
    "A201");

  savo_locations::CandidateRecordData
    pending_record;

  pending_record.state =
    savo_locations::CandidateState::
      kPendingReview;

  pending_record.candidate_revision = 3U;
  pending_record.candidate = pending;

  ASSERT_TRUE(
    catalog.restore_candidate_record(
      pending_record).success);

  auto location = make_location(
    "B101",
    "campus_main",
    7U,
    28);

  location.source_candidate_id =
    "candidate-approved";

  ASSERT_TRUE(
    catalog.insert_location(location).success);

  auto approved = make_candidate(
    "candidate-approved",
    "campus_main",
    7U,
    28,
    "B101");

  savo_locations::CandidateRecordData
    approved_record;

  approved_record.state =
    savo_locations::CandidateState::kApproved;

  approved_record.candidate_revision = 2U;
  approved_record.candidate = approved;
  approved_record.review_reason = "approved";
  approved_record.approved_location_id = "B101";

  ASSERT_TRUE(
    catalog.restore_candidate_record(
      approved_record).success);

  auto rejected = make_candidate(
    "candidate-rejected",
    "campus_main",
    7U,
    29,
    "C301");

  savo_locations::CandidateRecordData
    rejected_record;

  rejected_record.state =
    savo_locations::CandidateState::kRejected;

  rejected_record.candidate_revision = 2U;
  rejected_record.candidate = rejected;
  rejected_record.review_reason = "bad geometry";

  ASSERT_TRUE(
    catalog.restore_candidate_record(
      rejected_record).success);

  EXPECT_EQ(catalog.candidate_size(), 3U);

  const auto restored_pending =
    catalog.get_candidate("candidate-pending");

  ASSERT_TRUE(restored_pending.has_value());
  EXPECT_EQ(restored_pending->candidate_revision, 3U);
}


TEST(LocationCatalog, RestoreRejectsInvalidEnvelope)
{
  savo_locations::InMemoryLocationCatalog catalog;

  savo_locations::CandidateRecordData record;
  record.state =
    savo_locations::CandidateState::kApproved;
  record.candidate_revision = 2U;
  record.candidate = make_candidate(
    "candidate-invalid",
    "campus_main",
    7U,
    27,
    "A201");

  const auto result =
    catalog.restore_candidate_record(record);

  EXPECT_FALSE(result.success);

  EXPECT_EQ(
    result.code,
    savo_locations::CandidateMutationCode::
      kInvalidCandidate);
}


TEST(LocationCatalog, ClearRemovesAllEntities)
{
  savo_locations::InMemoryLocationCatalog catalog;

  ASSERT_TRUE(
    catalog.insert_location(
      make_location(
        "A201",
        "campus_main",
        7U,
        27)).success);

  ASSERT_TRUE(
    catalog.register_candidate(
      make_candidate(
        "candidate-28",
        "campus_main",
        7U,
        28,
        "A202")).success);

  catalog.clear();

  EXPECT_EQ(catalog.location_size(), 0U);
  EXPECT_EQ(catalog.candidate_size(), 0U);
}


TEST(LocationCatalog, ResultStringsAreStable)
{
  using savo_locations::ApprovalCode;
  using savo_locations::CandidateMutationCode;
  using savo_locations::to_string;

  EXPECT_EQ(
    to_string(
      CandidateMutationCode::
        kEmptyReviewReason),
    "empty_review_reason");

  EXPECT_EQ(
    to_string(
      ApprovalCode::
        kMissingApproachPose),
    "missing_approach_pose");

  EXPECT_EQ(
    to_string(
      ApprovalCode::kLocationConflict),
    "location_conflict");
}
