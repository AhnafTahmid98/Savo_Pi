#include "savo_mapping/supervisor_runtime.hpp"

#include <cmath>
#include <string>

namespace savo_mapping
{


std::string validate_monitor_only_policy(
  const MonitorOnlyPolicy & policy)
{
  if (policy.allow_direct_motor_control) {
    return "direct_motor_control_must_be_disabled";
  }

  if (policy.allow_internal_package_calls) {
    return "internal_package_calls_must_be_disabled";
  }

  if (policy.navigation_handoff_enabled) {
    return "navigation_handoff_must_be_disabled";
  }

  if (policy.realsense_monitoring_enabled) {
    return "realsense_monitoring_not_implemented";
  }

  if (policy.voxel_obstacle_monitoring_enabled) {
    return "voxel_obstacle_monitoring_not_implemented";
  }

  if (policy.semantic_mapping_enabled) {
    return "semantic_mapping_not_implemented";
  }

  return {};
}

bool is_fresh_age(double age_s, double timeout_s)
{
  return std::isfinite(age_s) &&
         std::isfinite(timeout_s) &&
         age_s >= 0.0 &&
         timeout_s > 0.0 &&
         age_s <= timeout_s;
}

SupervisorDecision evaluate_supervisor_runtime(
  const SupervisorRequirements & requirements,
  const SupervisorInputs & inputs,
  const std::string & tf_failure_reason)
{
  SupervisorDecision decision;

  decision.slam_active =
    inputs.map_fresh && inputs.map_structure_valid;

  if (requirements.require_scan && !inputs.scan_fresh) {
    decision.reason = "not_ready: missing_or_stale_scan";
    return decision;
  }

  if (requirements.require_odom && !inputs.odom_fresh) {
    decision.reason = "not_ready: missing_or_stale_odom";
    return decision;
  }

  if (requirements.require_map && !inputs.map_fresh) {
    decision.reason = "not_ready: missing_or_stale_map";
    return decision;
  }

  if (requirements.require_map && !inputs.map_structure_valid) {
    decision.reason = "not_ready: invalid_map_structure";
    return decision;
  }

  if (requirements.require_tf && !inputs.tf_ok) {
    decision.reason = "not_ready: tf_not_ok";

    if (!tf_failure_reason.empty()) {
      decision.reason += ":" + tf_failure_reason;
    }

    return decision;
  }

  decision.ready = true;
  decision.reason = "ready";
  return decision;
}

}  // namespace savo_mapping
