// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_mapping/coverage_operation_orchestrator.hpp"

namespace coverage = savo_mapping::coverage;

TEST(CoverageOperationOrchestratorTest, AcceptsDefaultPolicy)
{
  EXPECT_TRUE(
    coverage::validate_coverage_operation_policy(
      coverage::CoverageOperationPolicy{}).empty());
}

TEST(CoverageOperationOrchestratorTest, ParsesSupervisorState)
{
  const std::string payload =
    R"({"schema_version":1,"node":"savo_supervisor",)"
    R"("lifecycle":"RUNNING","health":"OK","ready":true,)"
    R"("degraded":false,"reason_code":"supervisor_operational"})";
  const auto snapshot =
    coverage::parse_supervisor_authorization(payload);

  EXPECT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.ready);
  EXPECT_EQ(snapshot.lifecycle, "RUNNING");
  EXPECT_EQ(snapshot.health, "OK");
}

TEST(CoverageOperationOrchestratorTest, RejectsMalformedSupervisorState)
{
  const auto snapshot =
    coverage::parse_supervisor_authorization("not-json");
  EXPECT_FALSE(snapshot.valid);
}

TEST(CoverageOperationOrchestratorTest, ParsesHandoffState)
{
  const std::string payload =
    R"({"enabled":true,"state":"plan_available","reason":"idle",)"
    R"("candidate_valid":true,"candidate_generation":7,)"
    R"("candidate_age_sec":0.25,"mission_id":"",)"
    R"("terminal_state":"","result_reason":""})";
  const auto snapshot =
    coverage::parse_coverage_handoff_snapshot(payload);

  EXPECT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.enabled);
  EXPECT_TRUE(snapshot.candidate_valid);
  EXPECT_EQ(snapshot.candidate_generation, 7U);
}

TEST(CoverageOperationOrchestratorTest, ApprovesOnlyWithTwoKeys)
{
  coverage::SupervisorAuthorizationSnapshot supervisor;
  supervisor.valid = true;
  supervisor.ready = true;
  supervisor.lifecycle = "RUNNING";
  supervisor.health = "OK";
  supervisor.reason = "supervisor_operational";

  coverage::CoverageHandoffSnapshot handoff;
  handoff.valid = true;
  handoff.enabled = true;
  handoff.candidate_valid = true;
  handoff.candidate_generation = 9U;
  handoff.state = "plan_available";

  const auto decision = coverage::evaluate_coverage_approval(
    supervisor,
    0.1,
    handoff,
    1.0,
    coverage::CoverageOperationPolicy{});

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.candidate_generation, 9U);
}

TEST(CoverageOperationOrchestratorTest, RejectsStaleSupervisor)
{
  coverage::SupervisorAuthorizationSnapshot supervisor;
  supervisor.valid = true;
  supervisor.ready = true;
  supervisor.lifecycle = "RUNNING";
  supervisor.health = "OK";
  supervisor.reason = "supervisor_operational";

  coverage::CoverageHandoffSnapshot handoff;
  handoff.valid = true;
  handoff.enabled = true;
  handoff.candidate_valid = true;
  handoff.candidate_generation = 1U;
  handoff.state = "plan_available";

  const auto decision = coverage::evaluate_coverage_approval(
    supervisor,
    2.0,
    handoff,
    0.1,
    coverage::CoverageOperationPolicy{});

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(
    decision.reason,
    "coverage_operation_supervisor_stale");
}

TEST(CoverageOperationOrchestratorTest, RejectsPlanWithoutOperatorReadyState)
{
  coverage::SupervisorAuthorizationSnapshot supervisor;
  supervisor.valid = true;
  supervisor.ready = true;
  supervisor.lifecycle = "RUNNING";
  supervisor.health = "OK";
  supervisor.reason = "supervisor_operational";

  coverage::CoverageHandoffSnapshot handoff;
  handoff.valid = true;
  handoff.enabled = true;
  handoff.candidate_valid = true;
  handoff.candidate_generation = 3U;
  handoff.state = "executing";

  const auto decision = coverage::evaluate_coverage_approval(
    supervisor,
    0.1,
    handoff,
    0.1,
    coverage::CoverageOperationPolicy{});

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(
    decision.reason,
    "coverage_operation_no_approvable_plan");
}

TEST(CoverageOperationOrchestratorTest, MatchesMissionGeneration)
{
  EXPECT_TRUE(
    coverage::mission_id_matches_candidate_generation(
      "coverage-42-123456789",
      42U));
  EXPECT_FALSE(
    coverage::mission_id_matches_candidate_generation(
      "coverage-43-123456789",
      42U));
}

TEST(CoverageOperationOrchestratorTest, MapsReadyState)
{
  coverage::CoverageHandoffSnapshot handoff;
  handoff.valid = true;
  handoff.state = "plan_available";

  EXPECT_EQ(
    coverage::coverage_operation_effective_state(
      true, false, true, handoff),
    "ready_for_approval");
  EXPECT_EQ(
    coverage::coverage_operation_effective_state(
      true, false, false, handoff),
    "blocked_by_supervisor");
}
