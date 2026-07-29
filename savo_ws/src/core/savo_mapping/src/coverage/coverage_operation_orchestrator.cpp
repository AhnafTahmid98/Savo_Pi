// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_mapping/coverage_operation_orchestrator.hpp"

#include <cmath>
#include <string>

#include <nlohmann/json.hpp>

namespace savo_mapping::coverage
{
namespace
{

bool finite_positive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finite_nonnegative(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

std::string json_string(
  const nlohmann::json & value,
  const char * key)
{
  if (!value.contains(key) || !value.at(key).is_string()) {
    return {};
  }
  return value.at(key).get<std::string>();
}

bool json_bool(
  const nlohmann::json & value,
  const char * key,
  const bool fallback)
{
  if (!value.contains(key) || !value.at(key).is_boolean()) {
    return fallback;
  }
  return value.at(key).get<bool>();
}

}  // namespace

std::string validate_coverage_operation_policy(
  const CoverageOperationPolicy & policy)
{
  if (!finite_positive(policy.supervisor_timeout_sec)) {
    return "coverage_operation_supervisor_timeout_invalid";
  }
  if (!finite_positive(policy.maximum_candidate_age_sec)) {
    return "coverage_operation_candidate_age_invalid";
  }
  if (!finite_positive(policy.internal_service_timeout_sec)) {
    return "coverage_operation_internal_service_timeout_invalid";
  }
  if (!finite_nonnegative(
      policy.supervisor_loss_cancel_delay_sec))
  {
    return "coverage_operation_supervisor_loss_delay_invalid";
  }
  return {};
}

SupervisorAuthorizationSnapshot parse_supervisor_authorization(
  const std::string & payload)
{
  SupervisorAuthorizationSnapshot result;
  try {
    const auto json = nlohmann::json::parse(payload);
    if (!json.is_object() ||
      json.value("schema_version", 0) != 1 ||
      json_string(json, "node") != "savo_supervisor")
    {
      return result;
    }

    result.lifecycle = json_string(json, "lifecycle");
    result.health = json_string(json, "health");
    result.reason = json_string(json, "reason_code");
    result.ready = json_bool(json, "ready", false);
    result.degraded = json_bool(json, "degraded", false);
    result.valid =
      !result.lifecycle.empty() &&
      !result.health.empty() &&
      !result.reason.empty();
  } catch (const nlohmann::json::exception &) {
    return result;
  }
  return result;
}

CoverageHandoffSnapshot parse_coverage_handoff_snapshot(
  const std::string & payload)
{
  CoverageHandoffSnapshot result;
  try {
    const auto json = nlohmann::json::parse(payload);
    if (!json.is_object()) {
      return result;
    }

    result.enabled = json_bool(json, "enabled", false);
    result.candidate_valid =
      json_bool(json, "candidate_valid", false);
    result.state = json_string(json, "state");
    result.reason = json_string(json, "reason");
    result.mission_id = json_string(json, "mission_id");
    result.terminal_state =
      json_string(json, "terminal_state");
    result.result_reason =
      json_string(json, "result_reason");

    if (json.contains("candidate_generation") &&
      json.at("candidate_generation").is_number_unsigned())
    {
      result.candidate_generation =
        json.at("candidate_generation").get<std::uint64_t>();
    }
    if (json.contains("candidate_age_sec") &&
      json.at("candidate_age_sec").is_number())
    {
      result.candidate_age_sec =
        json.at("candidate_age_sec").get<double>();
    }

    result.valid =
      !result.state.empty() &&
      finite_nonnegative(result.candidate_age_sec);
  } catch (const nlohmann::json::exception &) {
    return result;
  }
  return result;
}

bool supervisor_authorized(
  const SupervisorAuthorizationSnapshot & snapshot,
  const CoverageOperationPolicy & policy)
{
  if (!snapshot.valid ||
    snapshot.lifecycle != "RUNNING" ||
    !snapshot.ready)
  {
    return false;
  }

  if (snapshot.health == "OK" && !snapshot.degraded) {
    return true;
  }

  return policy.allow_degraded_supervisor &&
         snapshot.health == "DEGRADED" &&
         snapshot.degraded;
}

CoverageApprovalDecision evaluate_coverage_approval(
  const SupervisorAuthorizationSnapshot & supervisor,
  const double supervisor_age_sec,
  const CoverageHandoffSnapshot & handoff,
  const double current_candidate_age_sec,
  const CoverageOperationPolicy & policy)
{
  CoverageApprovalDecision result;

  const auto policy_error =
    validate_coverage_operation_policy(policy);
  if (!policy_error.empty()) {
    result.reason = policy_error;
    return result;
  }
  if (!finite_nonnegative(supervisor_age_sec) ||
    supervisor_age_sec > policy.supervisor_timeout_sec)
  {
    result.reason = "coverage_operation_supervisor_stale";
    return result;
  }
  if (!supervisor_authorized(supervisor, policy)) {
    result.reason = supervisor.valid ?
      "coverage_operation_supervisor_not_authorized" :
      "coverage_operation_supervisor_invalid";
    return result;
  }
  if (!handoff.valid || !handoff.enabled) {
    result.reason = "coverage_operation_handoff_unavailable";
    return result;
  }
  if (handoff.state != "plan_available" ||
    !handoff.candidate_valid ||
    handoff.candidate_generation == 0U)
  {
    result.reason = "coverage_operation_no_approvable_plan";
    return result;
  }
  if (!finite_nonnegative(current_candidate_age_sec) ||
    current_candidate_age_sec >
    policy.maximum_candidate_age_sec)
  {
    result.reason = "coverage_operation_candidate_stale";
    return result;
  }

  result.accepted = true;
  result.reason = "coverage_operation_approval_allowed";
  result.candidate_generation =
    handoff.candidate_generation;
  return result;
}

bool coverage_handoff_state_active(const std::string & state)
{
  return state == "awaiting_dispatch" ||
         state == "executing" ||
         state == "canceling";
}

bool coverage_handoff_state_terminal(const std::string & state)
{
  return state == "succeeded" ||
         state == "canceled" ||
         state == "failed" ||
         state == "rejected" ||
         state == "timed_out";
}

bool mission_id_matches_candidate_generation(
  const std::string & mission_id,
  const std::uint64_t candidate_generation)
{
  if (mission_id.empty() || candidate_generation == 0U) {
    return false;
  }
  const auto marker =
    "-" + std::to_string(candidate_generation) + "-";
  return mission_id.find(marker) != std::string::npos;
}

std::string coverage_operation_effective_state(
  const bool enabled,
  const bool approval_pending,
  const bool supervisor_is_authorized,
  const CoverageHandoffSnapshot & handoff)
{
  if (!enabled) {
    return "disabled";
  }
  if (approval_pending) {
    return "approval_pending";
  }
  if (!handoff.valid) {
    return supervisor_is_authorized ?
           "waiting_for_plan" :
           "waiting_for_supervisor";
  }
  if (handoff.state == "plan_available") {
    return supervisor_is_authorized ?
           "ready_for_approval" :
           "blocked_by_supervisor";
  }
  if (handoff.state == "waiting_for_plan" ||
    handoff.state == "plan_invalid")
  {
    return supervisor_is_authorized ?
           handoff.state :
           "waiting_for_supervisor";
  }
  return handoff.state;
}

}  // namespace savo_mapping::coverage
