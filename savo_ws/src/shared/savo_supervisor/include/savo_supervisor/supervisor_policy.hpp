// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "savo_supervisor/component_status.hpp"
#include "savo_supervisor/supervisor_state.hpp"

namespace savo_supervisor
{

struct SupervisorPolicy
{
  double publish_rate_hz = 2.0;
  double startup_grace_s = 3.0;
  std::string state_summary_topic = "/savo_supervisor/state_summary";
  std::string heartbeat_topic = "/savo_supervisor/heartbeat";
  std::string health_topic = "/savo_supervisor/health";
  std::string events_topic = "/savo_supervisor/events";

  ComponentConfig localization;

  SupervisorPolicy();
  bool Validate() const;
  std::string ValidationError() const;

  static ComponentConfig DefaultLocalizationConfig();
  ComponentSummary EvaluateComponent(
    const ComponentStatus & status,
    const rclcpp::Time & now,
    const double startup_age_s) const;
  SupervisorState EvaluateSupervisor(
    const ComponentSummary & localization_summary,
    const rclcpp::Time & now,
    const double startup_age_s) const;
  std::string CompactStateJson(
    const SupervisorState & state,
    const rclcpp::Time & now) const;
  std::string CompactHeartbeatJson(
    const SupervisorState & state,
    const rclcpp::Time & now) const;
};

}  // namespace savo_supervisor
