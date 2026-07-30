// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_nav/location_navigation_contract.hpp"

namespace
{

savo_nav::LocationRecordView approved_record()
{
  savo_nav::LocationRecordView record;
  record.approved = true;
  record.enabled = true;
  record.location_id = "A201";
  record.map_id = "campus_main";
  record.map_revision = 7U;
  record.approach_pose = {"map", 3.2, 2.0, 0.0, true};
  record.tag_pose_map = {"map", 4.0, 2.0, 0.0, true};
  record.tag_pose_map_valid = true;
  record.arrival_confirmation_required = true;
  return record;
}

savo_nav::LocationNavigationRequestView request()
{
  savo_nav::LocationNavigationRequestView value;
  value.query = "room a201";
  value.enforce_map_context = true;
  value.map_id = "campus_main";
  value.map_revision = 7U;
  return value;
}

TEST(LocationNavigationContract, SelectsOnlyApprovedApproachPose)
{
  savo_nav::LocationNavigationContract contract;
  const auto decision = contract.SelectTarget(request(), approved_record());

  ASSERT_TRUE(decision.accepted);
  EXPECT_EQ(
    decision.target_source,
    savo_nav::LocationTargetSource::kApproachPose);
  EXPECT_DOUBLE_EQ(decision.target.x, 3.2);
  EXPECT_TRUE(decision.arrival_confirmation_required);
}

TEST(LocationNavigationContract, RejectsDirectTagPoseTarget)
{
  savo_nav::LocationNavigationContract contract;
  auto record = approved_record();
  record.approach_pose = record.tag_pose_map;

  const auto decision = contract.SelectTarget(request(), record);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(
    decision.code,
    savo_nav::LocationTargetCode::kDirectTagPoseTarget);
}

TEST(LocationNavigationContract, RejectsDisabledAndWrongMap)
{
  savo_nav::LocationNavigationContract contract;
  auto record = approved_record();
  record.enabled = false;
  EXPECT_EQ(
    contract.SelectTarget(request(), record).code,
    savo_nav::LocationTargetCode::kDisabled);

  record = approved_record();
  auto wrong_map = request();
  wrong_map.map_revision = 8U;
  EXPECT_EQ(
    contract.SelectTarget(wrong_map, record).code,
    savo_nav::LocationTargetCode::kMapMismatch);
}

}  // namespace
