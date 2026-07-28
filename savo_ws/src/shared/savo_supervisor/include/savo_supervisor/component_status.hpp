#pragma once

#include <string>

#include "savo_supervisor/freshness_tracker.hpp"
#include "savo_supervisor/supervisor_state.hpp"

namespace savo_supervisor
{

struct ComponentConfig
{
  std::string name;
  bool enabled = false;
  bool required = false;
  std::string health_topic;
  std::string summary_topic;
  std::string heartbeat_topic;
  double health_timeout_s = 0.0;
  double summary_timeout_s = 0.0;
  double heartbeat_timeout_s = 0.0;
  int expected_schema_version = 1;
};

struct ComponentStatus
{
  ComponentConfig config;
  FreshnessTracker health_tracker;
  FreshnessTracker summary_tracker;
  FreshnessTracker heartbeat_tracker;

  bool health_valid = false;
  bool summary_valid = false;
  bool heartbeat_valid = false;

  std::string health_state;
  bool health_ready = false;
  bool health_degraded = false;
  std::string health_reason_code;

  std::string summary_state;
  bool summary_ready = false;
  bool summary_degraded = false;
  std::string summary_reason_code;

  std::string heartbeat_state;
  bool heartbeat_alive = false;
  bool heartbeat_ready = false;
  std::string heartbeat_reason_code;
};

} // namespace savo_supervisor
