// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <chrono>

#include "savo_mapping/local_mapping_authority.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using savo_mapping::local_authority::Authority;
using savo_mapping::local_authority::Health;
using savo_mapping::local_authority::Identity;

Identity identity()
{
  return {"mission-1", "operator", "request-1", "floor_a", 1U, false};
}

Health health(const Clock::time_point now)
{
  Health result;
  result.observe(
    R"({"schema_version":1,"node":"savo_mapping","admission_ready":true,)"
    R"("continuation_ready":true,"semantic_ready":true,"reason":"ready"})", now);
  return result;
}
}  // namespace

TEST(LocalMappingAuthority, StartsUnownedAndAcquiresOnlyOnExplicitHealthyRequest)
{
  Authority authority;
  const auto now = Clock::time_point{};
  EXPECT_FALSE(authority.active());
  EXPECT_FALSE(authority.acquire(identity(), 0U, Health{}, now));
  EXPECT_TRUE(authority.acquire(identity(), 0U, health(now), now));
  EXPECT_TRUE(authority.active());
  EXPECT_GT(authority.generation(), 0U);
  EXPECT_FALSE(authority.acquire(identity(), 0U, health(now), now));
}

TEST(LocalMappingAuthority, RejectsForeignGenerationAndPreservesIdentityBinding)
{
  Authority authority;
  const auto now = Clock::time_point{};
  EXPECT_FALSE(authority.acquire(identity(), 42U, health(now), now));
  EXPECT_EQ(authority.reason(), "mapping_local_preacquired_generation_not_supported");
  ASSERT_TRUE(authority.acquire(identity(), 0U, health(now), now));
  EXPECT_TRUE(authority.check(identity(), authority.generation(), health(now), now));
  auto wrong = identity();
  wrong.actor_id = "other";
  EXPECT_FALSE(authority.check(wrong, authority.generation(), health(now), now));
  wrong = identity();
  wrong.map_revision = 2U;
  EXPECT_FALSE(authority.check(wrong, authority.generation(), health(now), now));
  EXPECT_FALSE(authority.check(identity(), authority.generation() + 1U, health(now), now));
}

TEST(LocalMappingAuthority, EnvironmentalInterlockContinuesButDoesNotAdmit)
{
  Authority authority;
  const auto now = Clock::time_point{};
  ASSERT_TRUE(authority.acquire(identity(), 0U, health(now), now));
  Health obstacle;
  obstacle.observe(
    R"({"schema_version":1,"node":"savo_mapping","admission_ready":false,)"
    R"("continuation_ready":true,"semantic_ready":true,"reason":"environmental_stop"})", now);
  EXPECT_TRUE(authority.revalidate(obstacle, now));
  EXPECT_TRUE(authority.active());
  Authority another;
  EXPECT_FALSE(another.acquire(identity(), 0U, obstacle, now));
  EXPECT_TRUE(authority.revalidate(health(now), now));
}

TEST(LocalMappingAuthority, StaleOrMalformedHealthRevokesAndDoesNotAutoRecover)
{
  const auto now = Clock::time_point{};
  for (const bool malformed : {false, true}) {
    Authority authority;
    auto observation = health(now);
    ASSERT_TRUE(authority.acquire(identity(), 0U, observation, now));
    if (malformed) {
      observation.observe("{}", now);
    }
    EXPECT_FALSE(authority.revalidate(observation, now + 1501ms));
    EXPECT_FALSE(authority.active());
    EXPECT_FALSE(authority.revalidate(health(now + 2s), now + 2s));
    EXPECT_FALSE(authority.active());
  }
}

TEST(LocalMappingAuthority, CriticalFailureReleaseAndPauseRequireExplicitAuthorityTransitions)
{
  const auto now = Clock::time_point{};
  Authority authority;
  ASSERT_TRUE(authority.acquire(identity(), 0U, health(now), now));
  authority.pause();
  EXPECT_FALSE(authority.active());
  EXPECT_TRUE(authority.owned());
  EXPECT_TRUE(authority.resume(health(now), now));
  Health critical;
  critical.observe(
    R"({"schema_version":1,"node":"savo_mapping","admission_ready":false,)"
    R"("continuation_ready":false,"semantic_ready":true,"reason":"core_ups_critical"})", now);
  EXPECT_FALSE(authority.revalidate(critical, now));
  EXPECT_EQ(authority.reason(), "core_ups_critical");
  authority.release();
  EXPECT_FALSE(authority.active());
  EXPECT_FALSE(authority.owned());
  EXPECT_FALSE(authority.resume(health(now), now));
  EXPECT_TRUE(authority.acquire(identity(), 0U, health(now), now));
}

TEST(LocalMappingAuthority, SemanticScopeAndHealthProducerIdentityAreNotIgnored)
{
  const auto now = Clock::time_point{};
  Health observation;
  observation.observe(
    R"({"schema_version":1,"node":"savo_mapping","admission_ready":true,)"
    R"("continuation_ready":true,"semantic_ready":false,"reason":"head_missing"})", now);
  auto semantic = identity();
  semantic.require_semantic = true;
  Authority authority;
  EXPECT_FALSE(authority.acquire(semantic, 0U, observation, now));
  EXPECT_TRUE(authority.acquire(identity(), 0U, observation, now));
  observation.observe(
    R"({"schema_version":1,"node":"savo_supervisor","admission_ready":true,)"
    R"("continuation_ready":true,"semantic_ready":true,"reason":"ready"})", now);
  EXPECT_FALSE(authority.revalidate(observation, now));
}
