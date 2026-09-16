// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_mapping/mapping_phase_authority.hpp"
#include <gtest/gtest.h>

namespace
{
using namespace std::chrono_literals;
using savo_mapping::phase_authority::Context;
using Clock = std::chrono::steady_clock;

std::string snapshot(bool active = true, bool semantic = true)
{
  return nlohmann::json{{"schema_version", 1}, {"node", "savo_mapping"},
    {"mission_id", "mission"}, {"actor_id", "operator"}, {"request_id", "lease"},
    {"map_id", "map"}, {"map_revision", 1}, {"generation", 3},
    {"require_semantic", true}, {"active", active}, {"coverage_allowed", active},
    {"semantic_allowed", semantic}}.dump();
}
}  // namespace

TEST(MappingPhaseAuthority, RequiresFreshExactParentIdentity)
{
  Context context;
  const auto now = Clock::time_point{};
  EXPECT_FALSE(context.coverage_ready(now));
  context.observe(snapshot(), now);
  EXPECT_TRUE(context.coverage_ready(now));
  EXPECT_TRUE(context.semantic_ready("operator", "map", 1U, now));
  EXPECT_FALSE(context.semantic_ready("intruder", "map", 1U, now));
  EXPECT_FALSE(context.semantic_ready("operator", "other", 1U, now));
  EXPECT_FALSE(context.semantic_ready("operator", "map", 2U, now));
  EXPECT_FALSE(context.coverage_ready(now + 1501ms));
  EXPECT_FALSE(context.semantic_ready("operator", "map", 1U, now + 1501ms));
  EXPECT_EQ(context.scoped_request_id("register"), "lease:3:register");
}

TEST(MappingPhaseAuthority, PausedSemanticPermissionDoesNotAuthorizeMotion)
{
  Context context;
  const auto now = Clock::time_point{};
  context.observe(snapshot(false), now);
  EXPECT_FALSE(context.coverage_ready(now));
  EXPECT_TRUE(context.semantic_ready("operator", "map", 1U, now));
  context.observe(snapshot(false, false), now);
  EXPECT_FALSE(context.semantic_ready("operator", "map", 1U, now));
}

TEST(MappingPhaseAuthority, CannotInheritAnotherMissionOrGeneration)
{
  Context first;
  const auto now = Clock::time_point{};
  first.observe(snapshot(), now);
  Context second;
  for (const auto * field : {"mission_id", "actor_id", "request_id", "map_id"}) {
    auto changed = nlohmann::json::parse(snapshot());
    changed[field] = "other";
    second.observe(changed.dump(), now);
    EXPECT_FALSE(first.same_lease(second));
  }
  auto changed = nlohmann::json::parse(snapshot());
  changed["generation"] = 4;
  second.observe(changed.dump(), now);
  EXPECT_FALSE(first.same_lease(second));
}

TEST(MappingPhaseAuthority, WrongProducerMalformedOrIncompleteEvidenceFailsClosed)
{
  Context context;
  const auto now = Clock::time_point{};
  for (const auto * field : {"node", "mission_id", "actor_id", "request_id", "map_id",
      "map_revision", "generation", "require_semantic", "active", "coverage_allowed"})
  {
    auto changed = nlohmann::json::parse(snapshot());
    changed.erase(field);
    context.observe(changed.dump(), now);
    EXPECT_FALSE(context.coverage_ready(now)) << field;
  }
  auto changed = nlohmann::json::parse(snapshot());
  changed["node"] = "savo_supervisor";
  context.observe(changed.dump(), now);
  EXPECT_FALSE(context.coverage_ready(now));
}
