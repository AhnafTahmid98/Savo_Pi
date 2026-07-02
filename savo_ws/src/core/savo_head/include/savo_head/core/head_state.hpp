#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "savo_head/core/head_types.hpp"

namespace savo_head
{

struct HeadRuntimeState
{
  PanTiltState pan_tilt{};
  bool driver_opened{false};
  bool dryrun{false};
  bool scanning{false};
  bool camera_streaming{false};
  bool apriltag_enabled{true};

  std::optional<std::string> last_error{};
  double last_command_s{0.0};
  double last_state_publish_s{0.0};
  double last_status_publish_s{0.0};

  [[nodiscard]] HeadRuntimeState normalized(const PanTiltLimits & limits = PanTiltLimits{}) const
  {
    HeadRuntimeState out = *this;
    out.pan_tilt = out.pan_tilt.normalized(limits);

    if (!std::isfinite(out.last_command_s) || out.last_command_s < 0.0) {
      out.last_command_s = 0.0;
    }

    if (!std::isfinite(out.last_state_publish_s) || out.last_state_publish_s < 0.0) {
      out.last_state_publish_s = 0.0;
    }

    if (!std::isfinite(out.last_status_publish_s) || out.last_status_publish_s < 0.0) {
      out.last_status_publish_s = 0.0;
    }

    if (out.last_error.has_value() && out.last_error->empty()) {
      out.last_error.reset();
    }

    return out;
  }

  [[nodiscard]] bool ok() const
  {
    return pan_tilt.status == HeadStatus::kOk && !last_error.has_value();
  }

  [[nodiscard]] bool has_error() const
  {
    return pan_tilt.status == HeadStatus::kError ||
      (last_error.has_value() && !last_error->empty());
  }

  [[nodiscard]] bool command_stale(double now_s, double timeout_s) const
  {
    if (last_command_s <= 0.0) {
      return true;
    }

    return (now_s - last_command_s) > timeout_s;
  }
};

struct HeadStateTransition
{
  PanTiltState previous{};
  PanTiltState current{};
  PanTiltCommand command{};
  bool accepted{false};
  bool clamped{false};
  std::vector<std::string> warnings{};
  std::optional<std::string> error{};

  [[nodiscard]] bool changed() const
  {
    return std::tie(previous.pan_deg, previous.tilt_deg, previous.mode) !=
      std::tie(current.pan_deg, current.tilt_deg, current.mode);
  }

  [[nodiscard]] bool ok() const
  {
    return accepted && !error.has_value();
  }
};

struct HeadStateDelta
{
  int pan_delta_deg{0};
  int tilt_delta_deg{0};
  bool mode_changed{false};
  bool status_changed{false};

  [[nodiscard]] bool any() const
  {
    return pan_delta_deg != 0 || tilt_delta_deg != 0 || mode_changed || status_changed;
  }
};

[[nodiscard]] HeadRuntimeState make_initial_head_state(
  const PanTiltLimits & limits = PanTiltLimits{},
  HeadStatus status = HeadStatus::kOk,
  bool dryrun = false,
  double stamp_s = 0.0);

[[nodiscard]] HeadRuntimeState normalize_head_state(
  const HeadRuntimeState & state,
  const PanTiltLimits & limits = PanTiltLimits{});

[[nodiscard]] HeadStateTransition apply_head_command_to_state(
  const PanTiltState & current,
  const PanTiltCommand & command,
  const PanTiltLimits & limits = PanTiltLimits{});

[[nodiscard]] HeadStateDelta state_delta(
  const PanTiltState & previous,
  const PanTiltState & current);

[[nodiscard]] bool pan_tilt_changed(
  const PanTiltState & previous,
  const PanTiltState & current);

[[nodiscard]] bool state_requires_publish(
  const PanTiltState & previous,
  const PanTiltState & current,
  double now_s,
  double last_publish_s,
  double max_publish_period_s);

[[nodiscard]] std::string state_summary(const PanTiltState & state);

[[nodiscard]] std::string runtime_state_summary(const HeadRuntimeState & state);

[[nodiscard]] std::optional<HeadMode> head_mode_from_string(const std::string & value);

[[nodiscard]] std::optional<HeadStatus> head_status_from_string(const std::string & value);

[[nodiscard]] HeadStatus status_from_opened_backend(bool opened, bool dryrun);

[[nodiscard]] inline PanTiltState make_center_state(
  double stamp_s = 0.0,
  HeadStatus status = HeadStatus::kOk,
  std::string source = "center")
{
  return PanTiltState{
    kPanCenterDeg,
    kTiltCenterDeg,
    HeadMode::kCentering,
    status,
    stamp_s,
    std::move(source)
  };
}

[[nodiscard]] inline PanTiltState make_idle_state(
  double stamp_s = 0.0,
  HeadStatus status = HeadStatus::kOk,
  std::string source = "idle")
{
  return PanTiltState{
    kPanCenterDeg,
    kTiltCenterDeg,
    HeadMode::kIdle,
    status,
    stamp_s,
    std::move(source)
  };
}

}  // namespace savo_head
