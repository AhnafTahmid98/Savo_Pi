#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "savo_head/core/head_types.hpp"

namespace savo_head
{

enum class DiagnosticLevel
{
  kOk,
  kWarn,
  kError
};

struct DiagnosticValue
{
  std::string key;
  std::string value;
};

struct ComponentHealth
{
  std::string name;
  HeadStatus status{HeadStatus::kOk};
  DiagnosticLevel level{DiagnosticLevel::kOk};
  std::string message{"OK"};
  std::vector<DiagnosticValue> values{};

  [[nodiscard]] bool ok() const
  {
    return level == DiagnosticLevel::kOk && status == HeadStatus::kOk;
  }

  void add_value(std::string key, std::string value)
  {
    values.push_back(DiagnosticValue{std::move(key), std::move(value)});
  }

  void add_value(const std::string & key, int value)
  {
    add_value(key, std::to_string(value));
  }

  void add_value(const std::string & key, double value)
  {
    std::ostringstream stream;
    stream << value;
    add_value(key, stream.str());
  }

  void add_value(const std::string & key, bool value)
  {
    add_value(key, std::string(value ? "true" : "false"));
  }
};

struct HeadHealthSummary
{
  bool ok{true};
  DiagnosticLevel level{DiagnosticLevel::kOk};
  std::string message{"OK"};
  std::vector<ComponentHealth> components{};

  [[nodiscard]] std::size_t error_count() const
  {
    return static_cast<std::size_t>(
      std::count_if(
        components.begin(),
        components.end(),
        [](const auto & item) {return item.level == DiagnosticLevel::kError;}));
  }

  [[nodiscard]] std::size_t warning_count() const
  {
    return static_cast<std::size_t>(
      std::count_if(
        components.begin(),
        components.end(),
        [](const auto & item) {return item.level == DiagnosticLevel::kWarn;}));
  }
};

struct StaleCheckResult
{
  bool stale{true};
  double age_s{0.0};
  std::string reason{};
};

[[nodiscard]] inline const char * to_string(DiagnosticLevel level)
{
  switch (level) {
    case DiagnosticLevel::kOk:
      return "OK";
    case DiagnosticLevel::kWarn:
      return "WARN";
    case DiagnosticLevel::kError:
      return "ERROR";
  }

  return "UNKNOWN";
}

[[nodiscard]] inline DiagnosticLevel level_from_status(HeadStatus status)
{
  switch (status) {
    case HeadStatus::kOk:
      return DiagnosticLevel::kOk;
    case HeadStatus::kDryrun:
    case HeadStatus::kStale:
    case HeadStatus::kDisabled:
      return DiagnosticLevel::kWarn;
    case HeadStatus::kError:
      return DiagnosticLevel::kError;
  }

  return DiagnosticLevel::kError;
}

[[nodiscard]] inline DiagnosticLevel combine_level(DiagnosticLevel lhs, DiagnosticLevel rhs)
{
  if (lhs == DiagnosticLevel::kError || rhs == DiagnosticLevel::kError) {
    return DiagnosticLevel::kError;
  }

  if (lhs == DiagnosticLevel::kWarn || rhs == DiagnosticLevel::kWarn) {
    return DiagnosticLevel::kWarn;
  }

  return DiagnosticLevel::kOk;
}

[[nodiscard]] inline ComponentHealth make_component_health(
  std::string name,
  HeadStatus status,
  std::string message = {})
{
  const auto level = level_from_status(status);

  if (message.empty()) {
    message = to_string(status);
  }

  return ComponentHealth{
    std::move(name),
    status,
    level,
    std::move(message),
    {}
  };
}

[[nodiscard]] inline ComponentHealth ok_component(std::string name, std::string message = "OK")
{
  return ComponentHealth{
    std::move(name),
    HeadStatus::kOk,
    DiagnosticLevel::kOk,
    std::move(message),
    {}
  };
}

[[nodiscard]] inline ComponentHealth warn_component(
  std::string name,
  HeadStatus status = HeadStatus::kStale,
  std::string message = "WARN")
{
  return ComponentHealth{
    std::move(name),
    status,
    DiagnosticLevel::kWarn,
    std::move(message),
    {}
  };
}

[[nodiscard]] inline ComponentHealth error_component(std::string name, std::string message)
{
  return ComponentHealth{
    std::move(name),
    HeadStatus::kError,
    DiagnosticLevel::kError,
    std::move(message),
    {}
  };
}

[[nodiscard]] inline StaleCheckResult check_stale(
  double now_s,
  double stamp_s,
  double timeout_s,
  bool required = true)
{
  StaleCheckResult result;

  if (!std::isfinite(now_s) || !std::isfinite(stamp_s) || !std::isfinite(timeout_s)) {
    result.stale = required;
    result.age_s = 0.0;
    result.reason = "non_finite_time";
    return result;
  }

  if (stamp_s <= 0.0) {
    result.stale = required;
    result.age_s = 0.0;
    result.reason = "never_seen";
    return result;
  }

  result.age_s = std::max(0.0, now_s - stamp_s);
  result.stale = result.age_s > timeout_s;
  result.reason = result.stale ? "stale" : "fresh";

  if (!required && result.reason == "never_seen") {
    result.stale = false;
  }

  return result;
}

[[nodiscard]] inline ComponentHealth health_from_stale_check(
  std::string name,
  const StaleCheckResult & stale,
  bool required)
{
  if (!stale.stale) {
    auto health = ok_component(std::move(name), "OK");
    health.add_value("age_s", stale.age_s);
    health.add_value("reason", stale.reason);
    health.add_value("required", required);
    return health;
  }

  auto health = warn_component(
    std::move(name),
    required ? HeadStatus::kStale : HeadStatus::kDryrun,
    required ? "STALE" : "DRYRUN");

  health.add_value("age_s", stale.age_s);
  health.add_value("reason", stale.reason);
  health.add_value("required", required);
  return health;
}

[[nodiscard]] inline HeadHealthSummary summarize_health(
  const std::vector<ComponentHealth> & components,
  bool warnings_are_ok = true)
{
  HeadHealthSummary summary;
  summary.components = components;
  summary.level = DiagnosticLevel::kOk;

  for (const auto & component : components) {
    summary.level = combine_level(summary.level, component.level);
  }

  const auto has_errors = summary.error_count() > 0U;
  const auto has_warnings = summary.warning_count() > 0U;

  summary.ok = !has_errors && (warnings_are_ok || !has_warnings);

  if (has_errors) {
    summary.message = "ERROR";
  } else if (has_warnings) {
    summary.message = warnings_are_ok ? "OK_WITH_WARNINGS" : "WARN";
  } else {
    summary.message = "OK";
  }

  return summary;
}

[[nodiscard]] inline std::string dashboard_text(const HeadHealthSummary & summary)
{
  std::ostringstream stream;
  stream << "savo_head status=" << summary.message
         << " errors=" << summary.error_count()
         << " warnings=" << summary.warning_count();

  for (const auto & component : summary.components) {
    stream << " " << component.name << "=" << to_string(component.status);
  }

  return stream.str();
}

[[nodiscard]] inline ComponentHealth pan_tilt_health(
  const PanTiltState & state,
  double now_s,
  double stale_timeout_s,
  bool required = true)
{
  const auto stale = check_stale(now_s, state.stamp_s, stale_timeout_s, required);
  auto health = health_from_stale_check("savo_head.pan_tilt", stale, required);

  if (!stale.stale && state.status != HeadStatus::kOk) {
    health.status = state.status;
    health.level = level_from_status(state.status);
    health.message = to_string(state.status);
  }

  health.add_value("pan_deg", state.pan_deg);
  health.add_value("tilt_deg", state.tilt_deg);
  health.add_value("mode", to_string(state.mode));
  health.add_value("source", state.source);

  return health;
}

[[nodiscard]] inline ComponentHealth hardware_health(
  bool opened,
  bool dryrun,
  const std::optional<std::string> & last_error = std::nullopt)
{
  if (last_error.has_value() && !last_error->empty()) {
    auto health = error_component("savo_head.hardware", *last_error);
    health.add_value("opened", opened);
    health.add_value("dryrun", dryrun);
    return health;
  }

  if (!opened) {
    auto health = warn_component("savo_head.hardware", HeadStatus::kStale, "NOT_OPEN");
    health.add_value("opened", opened);
    health.add_value("dryrun", dryrun);
    return health;
  }

  if (dryrun) {
    auto health = warn_component("savo_head.hardware", HeadStatus::kDryrun, "DRYRUN");
    health.add_value("opened", opened);
    health.add_value("dryrun", dryrun);
    return health;
  }

  auto health = ok_component("savo_head.hardware", "OK");
  health.add_value("opened", opened);
  health.add_value("dryrun", dryrun);
  return health;
}

[[nodiscard]] inline ComponentHealth scan_health(
  const std::string & state,
  const std::string & phase,
  int pan_deg,
  int tilt_deg,
  const std::optional<std::string> & error = std::nullopt)
{
  if (error.has_value() && !error->empty()) {
    auto health = error_component("savo_head.scan", *error);
    health.add_value("state", state);
    health.add_value("phase", phase);
    health.add_value("pan_deg", pan_deg);
    health.add_value("tilt_deg", tilt_deg);
    return health;
  }

  if (state == "running") {
    auto health = ok_component("savo_head.scan", "RUNNING");
    health.add_value("state", state);
    health.add_value("phase", phase);
    health.add_value("pan_deg", pan_deg);
    health.add_value("tilt_deg", tilt_deg);
    return health;
  }

  auto health = warn_component("savo_head.scan", HeadStatus::kDryrun, state);
  health.add_value("state", state);
  health.add_value("phase", phase);
  health.add_value("pan_deg", pan_deg);
  health.add_value("tilt_deg", tilt_deg);
  return health;
}

}  // namespace savo_head
