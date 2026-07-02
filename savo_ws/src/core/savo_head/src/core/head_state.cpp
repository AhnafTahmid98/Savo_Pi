#include "savo_head/core/head_state.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace savo_head
{

namespace
{

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});

  return value;
}

std::string join_errors(const std::vector<std::string> & errors)
{
  std::ostringstream stream;

  for (std::size_t i = 0; i < errors.size(); ++i) {
    if (i > 0U) {
      stream << "; ";
    }
    stream << errors[i];
  }

  return stream.str();
}

bool target_was_clamped(
  const PanTiltCommand & command,
  const PanTiltState & current,
  const PanTiltState & target,
  const PanTiltLimits & limits)
{
  if (command.type == CommandType::kCenter) {
    return false;
  }

  if (command.type == CommandType::kAbsolute) {
    if (command.pan_deg.has_value() && command.pan_deg.value() != target.pan_deg) {
      return true;
    }

    if (command.tilt_deg.has_value() && command.tilt_deg.value() != target.tilt_deg) {
      return true;
    }

    return false;
  }

  if (command.type == CommandType::kDelta) {
    const auto requested_pan = current.pan_deg + command.pan_delta_deg;
    const auto requested_tilt = current.tilt_deg + command.tilt_delta_deg;

    return requested_pan != limits.clamp_pan(clamp_servo_angle(requested_pan)) ||
      requested_tilt != limits.clamp_tilt(clamp_servo_angle(requested_tilt));
  }

  return false;
}

}  // namespace

HeadRuntimeState make_initial_head_state(
  const PanTiltLimits & limits,
  HeadStatus status,
  bool dryrun,
  double stamp_s)
{
  HeadRuntimeState state;
  state.pan_tilt = PanTiltState{
    limits.pan_center_deg,
    limits.tilt_center_deg,
    HeadMode::kIdle,
    status,
    stamp_s,
    "init"
  }.normalized(limits);

  state.driver_opened = false;
  state.dryrun = dryrun;
  state.scanning = false;
  state.camera_streaming = false;
  state.apriltag_enabled = true;
  state.last_command_s = stamp_s;
  state.last_state_publish_s = 0.0;
  state.last_status_publish_s = 0.0;

  return state.normalized(limits);
}

HeadRuntimeState normalize_head_state(
  const HeadRuntimeState & state,
  const PanTiltLimits & limits)
{
  return state.normalized(limits);
}

HeadStateTransition apply_head_command_to_state(
  const PanTiltState & current,
  const PanTiltCommand & command,
  const PanTiltLimits & limits)
{
  HeadStateTransition transition;
  transition.previous = current.normalized(limits);
  transition.command = command;

  const auto errors = command.validation_errors();
  if (!errors.empty()) {
    transition.current = transition.previous;
    transition.accepted = false;
    transition.error = join_errors(errors);
    return transition;
  }

  auto target = target_from_command(command, transition.previous, limits);
  target.status = transition.previous.status;

  if (command.type == CommandType::kStop) {
    target.mode = HeadMode::kIdle;
  } else if (command.type == CommandType::kHold) {
    target.mode = transition.previous.mode;
  } else if (command.type == CommandType::kDelta || command.type == CommandType::kAbsolute) {
    target.mode = HeadMode::kManual;
  }

  transition.current = target.normalized(limits);
  transition.accepted = true;
  transition.clamped = target_was_clamped(command, transition.previous, transition.current, limits);

  if (transition.clamped) {
    transition.warnings.emplace_back("target clamped to pan/tilt limits");
  }

  return transition;
}

HeadStateDelta state_delta(
  const PanTiltState & previous,
  const PanTiltState & current)
{
  return HeadStateDelta{
    current.pan_deg - previous.pan_deg,
    current.tilt_deg - previous.tilt_deg,
    previous.mode != current.mode,
    previous.status != current.status
  };
}

bool pan_tilt_changed(
  const PanTiltState & previous,
  const PanTiltState & current)
{
  return previous.pan_deg != current.pan_deg ||
    previous.tilt_deg != current.tilt_deg;
}

bool state_requires_publish(
  const PanTiltState & previous,
  const PanTiltState & current,
  double now_s,
  double last_publish_s,
  double max_publish_period_s)
{
  if (pan_tilt_changed(previous, current) ||
    previous.mode != current.mode ||
    previous.status != current.status)
  {
    return true;
  }

  if (!std::isfinite(now_s) || !std::isfinite(last_publish_s) ||
    !std::isfinite(max_publish_period_s))
  {
    return true;
  }

  if (last_publish_s <= 0.0) {
    return true;
  }

  return (now_s - last_publish_s) >= std::max(0.0, max_publish_period_s);
}

std::string state_summary(const PanTiltState & state)
{
  std::ostringstream stream;
  stream << "pan=" << state.pan_deg
         << ";tilt=" << state.tilt_deg
         << ";mode=" << to_string(state.mode)
         << ";status=" << to_string(state.status)
         << ";stamp_s=" << state.stamp_s
         << ";source=" << state.source;

  return stream.str();
}

std::string runtime_state_summary(const HeadRuntimeState & state)
{
  const auto normalized = state.normalized();

  std::ostringstream stream;
  stream << state_summary(normalized.pan_tilt)
         << ";driver_opened=" << (normalized.driver_opened ? "true" : "false")
         << ";dryrun=" << (normalized.dryrun ? "true" : "false")
         << ";scanning=" << (normalized.scanning ? "true" : "false")
         << ";camera_streaming=" << (normalized.camera_streaming ? "true" : "false")
         << ";apriltag_enabled=" << (normalized.apriltag_enabled ? "true" : "false");

  if (normalized.last_error.has_value()) {
    stream << ";last_error=" << normalized.last_error.value();
  }

  return stream.str();
}

std::optional<HeadMode> head_mode_from_string(const std::string & value)
{
  const auto text = lower_copy(value);

  if (text == "idle") {
    return HeadMode::kIdle;
  }

  if (text == "manual") {
    return HeadMode::kManual;
  }

  if (text == "auto") {
    return HeadMode::kAuto;
  }

  if (text == "centering") {
    return HeadMode::kCentering;
  }

  return std::nullopt;
}

std::optional<HeadStatus> head_status_from_string(const std::string & value)
{
  const auto text = lower_copy(value);

  if (text == "ok") {
    return HeadStatus::kOk;
  }

  if (text == "dryrun") {
    return HeadStatus::kDryrun;
  }

  if (text == "stale") {
    return HeadStatus::kStale;
  }

  if (text == "disabled") {
    return HeadStatus::kDisabled;
  }

  if (text == "error") {
    return HeadStatus::kError;
  }

  return std::nullopt;
}

HeadStatus status_from_opened_backend(bool opened, bool dryrun)
{
  if (!opened) {
    return HeadStatus::kStale;
  }

  return dryrun ? HeadStatus::kDryrun : HeadStatus::kOk;
}

}  // namespace savo_head
