#include <gtest/gtest.h>

#include "savo_locations/constants.hpp"
#include "savo_locations/service_names.hpp"
#include "savo_locations/topic_names.hpp"
#include "savo_locations/types.hpp"


TEST(LocationTypes, CandidateStateStringsAreStable)
{
  using savo_locations::CandidateState;
  using savo_locations::to_string;

  EXPECT_EQ(
    to_string(CandidateState::kUnknown),
    "unknown");

  EXPECT_EQ(
    to_string(CandidateState::kPendingReview),
    "pending_review");

  EXPECT_EQ(
    to_string(CandidateState::kApproved),
    "approved");

  EXPECT_EQ(
    to_string(CandidateState::kRejected),
    "rejected");
}


TEST(LocationTypes, CandidateTerminalPolicyIsStable)
{
  using savo_locations::CandidateState;
  using savo_locations::is_terminal;

  EXPECT_FALSE(
    is_terminal(CandidateState::kUnknown));

  EXPECT_FALSE(
    is_terminal(CandidateState::kPendingReview));

  EXPECT_TRUE(
    is_terminal(CandidateState::kApproved));

  EXPECT_TRUE(
    is_terminal(CandidateState::kRejected));
}


TEST(LocationTypes, NavigationRequiresApprovalAndEnablement)
{
  using savo_locations::LocationState;
  using savo_locations::is_navigable;

  EXPECT_TRUE(
    is_navigable(
      LocationState::kApproved,
      true));

  EXPECT_FALSE(
    is_navigable(
      LocationState::kApproved,
      false));

  EXPECT_FALSE(
    is_navigable(
      LocationState::kRetired,
      true));

  EXPECT_FALSE(
    is_navigable(
      LocationState::kUnknown,
      true));
}


TEST(LocationTypes, SemanticTypesRoundTrip)
{
  using savo_locations::SemanticType;
  using savo_locations::semantic_type_from_string;
  using savo_locations::to_string;

  const auto room =
    semantic_type_from_string("room");

  ASSERT_TRUE(room.has_value());
  EXPECT_EQ(room.value(), SemanticType::kRoom);
  EXPECT_EQ(to_string(room.value()), "room");

  const auto charging_station =
    semantic_type_from_string(
      "charging_station");

  ASSERT_TRUE(charging_station.has_value());

  EXPECT_EQ(
    charging_station.value(),
    SemanticType::kChargingStation);

  EXPECT_FALSE(
    semantic_type_from_string(
      "unsupported_type").has_value());
}


TEST(LocationTypes, ContractNamesAreAbsolute)
{
  using namespace savo_locations;

  EXPECT_EQ(
    service_names::kResolve,
    "/savo_locations/resolve");

  EXPECT_EQ(
    service_names::kRegisterCandidate,
    "/savo_locations/candidates/register");

  EXPECT_EQ(
    topic_names::kEvents,
    "/savo_locations/events");

  EXPECT_EQ(kSchemaVersion, 1U);
  EXPECT_EQ(kCanonicalMapFrame, "map");
}
