#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "savo_head/core/diagnostics.hpp"
#include "savo_head/core/head_state.hpp"
#include "savo_head/core/head_types.hpp"

namespace savo_head
{

inline constexpr const char * kHeadComponentController = "controller";
inline constexpr const char * kHeadComponentScan = "scan";
inline constexpr const char * kHeadComponentTf = "tf";
inline constexpr const char * kHeadComponentStatus = "status";
inline constexpr const char * kHeadComponentCamera = "camera";
inline constexpr const char * kHeadComponentAprilTag = "apriltag";

struct HeadComponentWatch
{
  std::string name;
  bool required{false};
  double stale_timeout_s{1.0};
  double last_stamp_s{0.0};
  std::string last_text{};
  bool seen{false};

  void update(double stamp_s, std::string text = {})
  {
    last_stamp_s = std::isfinite(stamp_s) && stamp_s >= 0.0 ? stamp_s : 0.0;
    last_text = std::move(text);
    seen = true;
  }

  void clear()
  {
    last_stamp_s = 0.0;
    last_text.clear();
    seen = false;
  }

  [[nodiscard]] double age_s(double now_s) const
  {
    if (!seen || last_stamp_s <= 0.0 || !std::isfinite(now_s)) {
      return 0.0;
    }

    return std::max(0.0, now_s - last_stamp_s);
  }

  [[nodiscard]] bool stale(double now_s) const
  {
    if (!seen) {
      return required;
    }

    return age_s(now_s) > stale_timeout_s;
  }

  [[nodiscard]] HeadStatus status(double now_s) const
  {
    if (!seen) {
      return required ? HeadStatus::kStale : HeadStatus::kDryrun;
    }

    if (age_s(now_s) > stale_timeout_s) {
      return required ? HeadStatus::kStale : HeadStatus::kDryrun;
    }

    return HeadStatus::kOk;
  }

  [[nodiscard]] ComponentHealth health(double now_s) const
  {
    auto item = make_component_health(
      "savo_head." + name,
      status(now_s),
      to_string(status(now_s)));

    item.add_value("required", required);
    item.add_value("seen", seen);
    item.add_value("age_s", age_s(now_s));
    item.add_value("stale_timeout_s", stale_timeout_s);
    item.add_value("last_text", last_text);

    return item;
  }

  [[nodiscard]] HeadComponentWatch normalized() const
  {
    HeadComponentWatch out = *this;

    if (out.name.empty()) {
      out.name = "unknown";
    }

    if (!std::isfinite(out.stale_timeout_s) || out.stale_timeout_s <= 0.0) {
      out.stale_timeout_s = 1.0;
    }

    if (!std::isfinite(out.last_stamp_s) || out.last_stamp_s < 0.0) {
      out.last_stamp_s = 0.0;
    }

    return out;
  }
};

struct HeadStatusSnapshot
{
  HeadComponentWatch controller{kHeadComponentController, true, 0.50};
  HeadComponentWatch scan{kHeadComponentScan, false, 1.00};
  HeadComponentWatch tf{kHeadComponentTf, false, 1.00};
  HeadComponentWatch camera{kHeadComponentCamera, false, 2.00};
  HeadComponentWatch apriltag{kHeadComponentAprilTag, false, 1.00};

  std::optional<std::string> last_error{};
  double stamp_s{0.0};

  [[nodiscard]] std::vector<ComponentHealth> component_health(double now_s) const
  {
    std::vector<ComponentHealth> components{
      controller.normalized().health(now_s),
      scan.normalized().health(now_s),
      tf.normalized().health(now_s),
      camera.normalized().health(now_s),
      apriltag.normalized().health(now_s)
    };

    if (last_error.has_value() && !last_error->empty()) {
      components.push_back(error_component("savo_head.status", *last_error));
    }

    return components;
  }

  [[nodiscard]] HeadHealthSummary summary(double now_s, bool warnings_are_ok = false) const
  {
    return summarize_health(component_health(now_s), warnings_are_ok);
  }

  [[nodiscard]] bool ok(double now_s, bool warnings_are_ok = false) const
  {
    return summary(now_s, warnings_are_ok).ok;
  }

  [[nodiscard]] HeadStatusSnapshot normalized() const
  {
    HeadStatusSnapshot out = *this;

    out.controller = out.controller.normalized();
    out.scan = out.scan.normalized();
    out.tf = out.tf.normalized();
    out.camera = out.camera.normalized();
    out.apriltag = out.apriltag.normalized();

    if (out.last_error.has_value() && out.last_error->empty()) {
      out.last_error.reset();
    }

    if (!std::isfinite(out.stamp_s) || out.stamp_s < 0.0) {
      out.stamp_s = 0.0;
    }

    return out;
  }
};

struct HeadStatusPolicy
{
  bool require_controller{true};
  bool require_scan{false};
  bool require_tf{false};
  bool require_camera{false};
  bool require_apriltag{false};

  double controller_stale_timeout_s{0.50};
  double scan_stale_timeout_s{1.00};
  double tf_stale_timeout_s{1.00};
  double camera_stale_timeout_s{2.00};
  double apriltag_stale_timeout_s{1.00};

  bool warnings_are_ok{false};

  [[nodiscard]] HeadStatusPolicy normalized() const
  {
    HeadStatusPolicy out = *this;

    out.controller_stale_timeout_s = normalize_timeout(out.controller_stale_timeout_s, 0.50);
    out.scan_stale_timeout_s = normalize_timeout(out.scan_stale_timeout_s, 1.00);
    out.tf_stale_timeout_s = normalize_timeout(out.tf_stale_timeout_s, 1.00);
    out.camera_stale_timeout_s = normalize_timeout(out.camera_stale_timeout_s, 2.00);
    out.apriltag_stale_timeout_s = normalize_timeout(out.apriltag_stale_timeout_s, 1.00);

    return out;
  }

private:
  [[nodiscard]] static double normalize_timeout(double value, double fallback)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      return fallback;
    }

    return value;
  }
};

[[nodiscard]] inline HeadStatusSnapshot make_status_snapshot_from_policy(
  const HeadStatusPolicy & policy,
  double stamp_s = 0.0)
{
  const auto item = policy.normalized();

  HeadStatusSnapshot snapshot;
  snapshot.controller.required = item.require_controller;
  snapshot.controller.stale_timeout_s = item.controller_stale_timeout_s;

  snapshot.scan.required = item.require_scan;
  snapshot.scan.stale_timeout_s = item.scan_stale_timeout_s;

  snapshot.tf.required = item.require_tf;
  snapshot.tf.stale_timeout_s = item.tf_stale_timeout_s;

  snapshot.camera.required = item.require_camera;
  snapshot.camera.stale_timeout_s = item.camera_stale_timeout_s;

  snapshot.apriltag.required = item.require_apriltag;
  snapshot.apriltag.stale_timeout_s = item.apriltag_stale_timeout_s;

  snapshot.stamp_s = stamp_s;

  return snapshot.normalized();
}

[[nodiscard]] inline std::string component_status_line(
  const HeadStatusSnapshot & snapshot,
  double now_s)
{
  const auto item = snapshot.normalized();

  std::ostringstream stream;
  stream << "controller=" << to_string(item.controller.status(now_s))
         << " scan=" << to_string(item.scan.status(now_s))
         << " tf=" << to_string(item.tf.status(now_s))
         << " camera=" << to_string(item.camera.status(now_s))
         << " apriltag=" << to_string(item.apriltag.status(now_s));

  return stream.str();
}

[[nodiscard]] inline std::string head_status_dashboard_text(
  const HeadStatusSnapshot & snapshot,
  double now_s,
  bool warnings_are_ok = false)
{
  const auto summary = snapshot.summary(now_s, warnings_are_ok);

  std::ostringstream stream;
  stream << "savo_head status=" << summary.message << " "
         << component_status_line(snapshot, now_s);

  return stream.str();
}

[[nodiscard]] inline ComponentHealth runtime_state_health(
  const HeadRuntimeState & runtime,
  double now_s,
  double stale_timeout_s)
{
  auto state = runtime.normalized();

  auto health = pan_tilt_health(
    state.pan_tilt,
    now_s,
    stale_timeout_s,
    true);

  health.name = "savo_head.runtime";
  health.add_value("driver_opened", state.driver_opened);
  health.add_value("dryrun", state.dryrun);
  health.add_value("scanning", state.scanning);
  health.add_value("camera_streaming", state.camera_streaming);
  health.add_value("apriltag_enabled", state.apriltag_enabled);

  if (state.last_error.has_value()) {
    health.level = DiagnosticLevel::kError;
    health.status = HeadStatus::kError;
    health.message = *state.last_error;
    health.add_value("last_error", *state.last_error);
  }

  return health;
}

[[nodiscard]] inline HeadHealthSummary runtime_health_summary(
  const HeadRuntimeState & runtime,
  double now_s,
  double stale_timeout_s,
  bool warnings_are_ok = true)
{
  return summarize_health(
    {runtime_state_health(runtime, now_s, stale_timeout_s)},
    warnings_are_ok);
}

}  // namespace savo_head
