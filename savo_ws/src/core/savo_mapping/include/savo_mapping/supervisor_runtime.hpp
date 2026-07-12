#pragma once

#include <string>

namespace savo_mapping
{

struct SupervisorRequirements
{
  bool require_scan{true};
  bool require_map{true};
  bool require_odom{true};
  bool require_tf{true};
};

struct SupervisorInputs
{
  bool scan_fresh{false};
  bool map_fresh{false};
  bool odom_fresh{false};
  bool tf_ok{false};
  bool map_structure_valid{false};
};

struct SupervisorDecision
{
  bool ready{false};
  bool slam_active{false};
  std::string reason{"not_ready: startup"};
};


struct MonitorOnlyPolicy
{
  bool allow_direct_motor_control{false};
  bool allow_internal_package_calls{false};
  bool navigation_handoff_enabled{false};

  bool realsense_monitoring_enabled{false};
  bool voxel_obstacle_monitoring_enabled{false};
  bool semantic_mapping_enabled{false};
};

std::string validate_monitor_only_policy(
  const MonitorOnlyPolicy & policy);

bool is_fresh_age(double age_s, double timeout_s);

SupervisorDecision evaluate_supervisor_runtime(
  const SupervisorRequirements & requirements,
  const SupervisorInputs & inputs,
  const std::string & tf_failure_reason = {});

}  // namespace savo_mapping
