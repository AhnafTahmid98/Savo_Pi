#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "savo_head/core/head_types.hpp"

namespace savo_head
{

inline constexpr const char * kScanModeStaged = "staged";

inline constexpr const char * kScanStateIdle = "idle";
inline constexpr const char * kScanStateRunning = "running";
inline constexpr const char * kScanStatePaused = "paused";
inline constexpr const char * kScanStateDone = "done";
inline constexpr const char * kScanStateError = "error";

inline constexpr const char * kScanPhasePan = "pan";
inline constexpr const char * kScanPhaseTiltSweep = "tilt_sweep";

inline constexpr int kScanTiltDirectionUp = 1;
inline constexpr int kScanTiltDirectionDown = -1;

inline constexpr int kScanPanStepDeg = 2;
inline constexpr int kScanTiltStepDeg = 2;
inline constexpr double kScanStepDelayS = 0.12;

struct ScanProfile
{
  std::string name{"semantic_scan"};
  bool enabled{true};
  std::string mode{kScanModeStaged};

  int pan_min_deg{kPanMinDeg};
  int pan_center_deg{kPanCenterDeg};
  int pan_max_deg{kPanMaxDeg};

  int tilt_min_deg{kTiltMinDeg};
  int tilt_max_deg{kTiltMaxDeg};

  int pan_step_deg{kScanPanStepDeg};
  int tilt_step_deg{kScanTiltStepDeg};
  double step_delay_s{kScanStepDelayS};

  int start_pan_deg{kPanMinDeg};
  int start_tilt_deg{kTiltMinDeg};

  std::vector<int> pan_targets_deg{kPanCenterDeg, kPanMaxDeg, kPanCenterDeg, kPanMinDeg};
  std::vector<int> tilt_sweep_pan_targets_deg{kPanCenterDeg};

  double hold_at_pan_target_s{0.10};
  double hold_after_tilt_sweep_s{0.15};

  bool pause_on_manual_command{true};
  double resume_after_manual_s{0.0};
  bool center_on_stop{true};

  [[nodiscard]] ScanProfile normalized() const
  {
    ScanProfile out = *this;

    if (out.mode != kScanModeStaged) {
      out.mode = kScanModeStaged;
    }

    if (out.pan_min_deg > out.pan_max_deg) {
      std::swap(out.pan_min_deg, out.pan_max_deg);
    }

    if (out.tilt_min_deg > out.tilt_max_deg) {
      std::swap(out.tilt_min_deg, out.tilt_max_deg);
    }

    out.pan_center_deg = std::clamp(out.pan_center_deg, out.pan_min_deg, out.pan_max_deg);
    out.pan_step_deg = std::max(1, out.pan_step_deg);
    out.tilt_step_deg = std::max(1, out.tilt_step_deg);
    out.step_delay_s = std::max(0.01, out.step_delay_s);

    out.start_pan_deg = std::clamp(out.start_pan_deg, out.pan_min_deg, out.pan_max_deg);
    out.start_tilt_deg = std::clamp(out.start_tilt_deg, out.tilt_min_deg, out.tilt_max_deg);

    if (out.pan_targets_deg.empty()) {
      out.pan_targets_deg = {out.pan_center_deg};
    }

    for (auto & value : out.pan_targets_deg) {
      value = std::clamp(value, out.pan_min_deg, out.pan_max_deg);
    }

    for (auto & value : out.tilt_sweep_pan_targets_deg) {
      value = std::clamp(value, out.pan_min_deg, out.pan_max_deg);
    }

    out.hold_at_pan_target_s = std::max(0.0, out.hold_at_pan_target_s);
    out.hold_after_tilt_sweep_s = std::max(0.0, out.hold_after_tilt_sweep_s);
    out.resume_after_manual_s = std::max(0.0, out.resume_after_manual_s);

    return out;
  }

  [[nodiscard]] bool needs_tilt_sweep_at(int pan_deg) const
  {
    const auto item = normalized();

    return std::find(
      item.tilt_sweep_pan_targets_deg.begin(),
      item.tilt_sweep_pan_targets_deg.end(),
      pan_deg) != item.tilt_sweep_pan_targets_deg.end();
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    if (mode != kScanModeStaged) {
      errors.emplace_back("unsupported scan mode");
    }

    if (pan_targets_deg.empty()) {
      errors.emplace_back("pan_targets_deg must not be empty");
    }

    if (pan_min_deg >= pan_max_deg) {
      errors.emplace_back("pan_min_deg must be lower than pan_max_deg");
    }

    if (tilt_min_deg >= tilt_max_deg) {
      errors.emplace_back("tilt_min_deg must be lower than tilt_max_deg");
    }

    if (pan_step_deg <= 0) {
      errors.emplace_back("pan_step_deg must be positive");
    }

    if (tilt_step_deg <= 0) {
      errors.emplace_back("tilt_step_deg must be positive");
    }

    if (step_delay_s <= 0.0) {
      errors.emplace_back("step_delay_s must be positive");
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }
};

struct ScanStatus
{
  std::string state{kScanStateIdle};
  std::string phase{kScanPhasePan};

  int pan_deg{kPanMinDeg};
  int tilt_deg{kTiltMinDeg};

  std::size_t pan_target_index{0};
  int current_pan_target_deg{kPanCenterDeg};
  int tilt_direction{kScanTiltDirectionUp};

  int cycle_count{0};
  int step_count{0};

  double stamp_s{0.0};
  double last_transition_s{0.0};
  std::string error{};

  [[nodiscard]] ScanStatus normalized(const ScanProfile & profile = ScanProfile{}) const
  {
    const auto item = profile.normalized();
    ScanStatus out = *this;

    if (
      out.state != kScanStateIdle &&
      out.state != kScanStateRunning &&
      out.state != kScanStatePaused &&
      out.state != kScanStateDone &&
      out.state != kScanStateError)
    {
      out.state = kScanStateIdle;
    }

    if (out.phase != kScanPhasePan && out.phase != kScanPhaseTiltSweep) {
      out.phase = kScanPhasePan;
    }

    out.pan_deg = std::clamp(out.pan_deg, item.pan_min_deg, item.pan_max_deg);
    out.tilt_deg = std::clamp(out.tilt_deg, item.tilt_min_deg, item.tilt_max_deg);

    out.pan_target_index %= item.pan_targets_deg.size();
    out.current_pan_target_deg = item.pan_targets_deg[out.pan_target_index];

    if (out.tilt_direction != kScanTiltDirectionDown) {
      out.tilt_direction = kScanTiltDirectionUp;
    }

    out.cycle_count = std::max(0, out.cycle_count);
    out.step_count = std::max(0, out.step_count);

    if (out.stamp_s < 0.0) {
      out.stamp_s = 0.0;
    }

    if (out.last_transition_s < 0.0) {
      out.last_transition_s = 0.0;
    }

    return out;
  }

  [[nodiscard]] bool running() const
  {
    return state == kScanStateRunning;
  }

  [[nodiscard]] bool paused() const
  {
    return state == kScanStatePaused;
  }

  [[nodiscard]] bool stopped() const
  {
    return state == kScanStateIdle || state == kScanStateDone || state == kScanStateError;
  }

  [[nodiscard]] int current_target(const ScanProfile & profile = ScanProfile{}) const
  {
    const auto item = profile.normalized();
    return item.pan_targets_deg[pan_target_index % item.pan_targets_deg.size()];
  }
};

struct ScanStepResult
{
  ScanStatus status;
  PanTiltCommand command;
};

[[nodiscard]] inline int move_toward(int current, int target, int step)
{
  const auto safe_step = std::max(1, step);

  if (current < target) {
    return std::min(current + safe_step, target);
  }

  if (current > target) {
    return std::max(current - safe_step, target);
  }

  return current;
}

[[nodiscard]] inline ScanStatus start_scan(
  const ScanProfile & profile = ScanProfile{},
  double stamp_s = 0.0)
{
  const auto item = profile.normalized();

  ScanStatus status;
  status.state = kScanStateRunning;
  status.phase = kScanPhasePan;
  status.pan_deg = item.start_pan_deg;
  status.tilt_deg = item.start_tilt_deg;
  status.pan_target_index = 0;
  status.current_pan_target_deg = item.pan_targets_deg.front();
  status.tilt_direction = kScanTiltDirectionUp;
  status.stamp_s = stamp_s;
  status.last_transition_s = stamp_s;

  return status;
}

[[nodiscard]] inline ScanStatus pause_scan(const ScanStatus & status, double stamp_s = 0.0)
{
  auto out = status.normalized();
  out.state = kScanStatePaused;
  out.stamp_s = stamp_s;
  out.last_transition_s = stamp_s;
  return out;
}

[[nodiscard]] inline ScanStatus resume_scan(const ScanStatus & status, double stamp_s = 0.0)
{
  auto out = status.normalized();
  out.state = kScanStateRunning;
  out.stamp_s = stamp_s;
  out.last_transition_s = stamp_s;
  return out;
}

[[nodiscard]] inline ScanStatus stop_scan(
  const ScanStatus & status,
  double stamp_s = 0.0,
  std::string error = {})
{
  auto out = status.normalized();
  out.state = error.empty() ? kScanStateDone : kScanStateError;
  out.stamp_s = stamp_s;
  out.last_transition_s = stamp_s;
  out.error = std::move(error);
  return out;
}

[[nodiscard]] inline std::tuple<std::size_t, int> next_pan_target_index(
  const ScanProfile & profile,
  std::size_t index)
{
  const auto item = profile.normalized();
  const auto next_index = (index + 1U) % item.pan_targets_deg.size();
  const auto wrapped = next_index == 0U ? 1 : 0;
  return {next_index, wrapped};
}

[[nodiscard]] inline ScanStepResult advance_scan_step(
  const ScanProfile & profile,
  const ScanStatus & status,
  double stamp_s = 0.0)
{
  const auto item = profile.normalized();
  auto state = status.normalized(item);

  if (!item.enabled) {
    auto stopped = stop_scan(state, stamp_s, "scan_disabled");
    return {stopped, hold_command(stamp_s, CommandSource::kScan, "scan_disabled")};
  }

  if (state.state == kScanStateIdle || state.state == kScanStateDone) {
    auto started = start_scan(item, stamp_s);
    auto command = absolute_command(
      started.pan_deg,
      started.tilt_deg,
      stamp_s,
      CommandSource::kScan,
      "scan_start");
    return {started, command};
  }

  if (state.state == kScanStatePaused) {
    return {state, hold_command(stamp_s, CommandSource::kScan, "scan_paused")};
  }

  if (state.state == kScanStateError) {
    return {state, hold_command(stamp_s, CommandSource::kScan, "scan_error")};
  }

  const auto target = state.current_target(item);

  if (state.phase == kScanPhasePan) {
    const auto new_pan = move_toward(state.pan_deg, target, item.pan_step_deg);

    state.pan_deg = new_pan;
    state.current_pan_target_deg = target;
    state.step_count += 1;
    state.stamp_s = stamp_s;

    if (new_pan == target) {
      if (item.needs_tilt_sweep_at(target)) {
        state.phase = kScanPhaseTiltSweep;
        state.tilt_deg = item.tilt_min_deg;
        state.tilt_direction = kScanTiltDirectionUp;
        state.last_transition_s = stamp_s;

        return {
          state,
          absolute_command(
            state.pan_deg,
            state.tilt_deg,
            stamp_s,
            CommandSource::kScan,
            "scan_tilt_start")
        };
      }

      auto [next_index, wrapped] = next_pan_target_index(item, state.pan_target_index);
      state.pan_target_index = next_index;
      state.current_pan_target_deg = item.pan_targets_deg[next_index];
      state.cycle_count += wrapped;
      state.last_transition_s = stamp_s;
    }

    return {
      state,
      absolute_command(
        state.pan_deg,
        state.tilt_deg,
        stamp_s,
        CommandSource::kScan,
        "scan_pan")
    };
  }

  auto next_tilt = state.tilt_deg + state.tilt_direction * item.tilt_step_deg;
  auto direction = state.tilt_direction;
  auto phase = std::string{kScanPhaseTiltSweep};
  auto next_index = state.pan_target_index;
  auto cycle_count = state.cycle_count;

  if (next_tilt >= item.tilt_max_deg) {
    next_tilt = item.tilt_max_deg;
    direction = kScanTiltDirectionDown;
  } else if (next_tilt <= item.tilt_min_deg) {
    next_tilt = item.tilt_min_deg;
    phase = kScanPhasePan;

    auto [new_index, wrapped] = next_pan_target_index(item, state.pan_target_index);
    next_index = new_index;
    cycle_count += wrapped;
  }

  state.phase = phase;
  state.tilt_deg = next_tilt;
  state.tilt_direction = direction;
  state.pan_target_index = next_index;
  state.current_pan_target_deg = item.pan_targets_deg[next_index];
  state.cycle_count = cycle_count;
  state.step_count += 1;
  state.stamp_s = stamp_s;

  if (state.phase == kScanPhasePan) {
    state.last_transition_s = stamp_s;
  }

  return {
    state,
    absolute_command(
      state.pan_deg,
      state.tilt_deg,
      stamp_s,
      CommandSource::kScan,
      "scan_tilt")
  };
}

struct ScanRuntime
{
  ScanProfile profile{};
  ScanStatus status{};
  std::optional<PanTiltCommand> last_command{};

  [[nodiscard]] ScanRuntime normalized() const
  {
    ScanRuntime out = *this;
    out.profile = out.profile.normalized();
    out.status = out.status.normalized(out.profile);
    return out;
  }

  [[nodiscard]] bool running() const
  {
    return status.state == kScanStateRunning;
  }

  [[nodiscard]] bool paused() const
  {
    return status.state == kScanStatePaused;
  }

  [[nodiscard]] bool stopped() const
  {
    return status.stopped();
  }

  [[nodiscard]] ScanRuntime start(double stamp_s = 0.0) const
  {
    ScanRuntime out = *this;
    out.profile = out.profile.normalized();
    out.status = start_scan(out.profile, stamp_s);
    out.last_command.reset();
    return out;
  }

  [[nodiscard]] ScanRuntime pause(double stamp_s = 0.0) const
  {
    ScanRuntime out = normalized();
    out.status = pause_scan(out.status, stamp_s);
    return out;
  }

  [[nodiscard]] ScanRuntime resume(double stamp_s = 0.0) const
  {
    ScanRuntime out = normalized();
    out.status = resume_scan(out.status, stamp_s);
    return out;
  }

  [[nodiscard]] ScanRuntime stop(double stamp_s = 0.0, std::string error = {}) const
  {
    ScanRuntime out = normalized();
    out.status = stop_scan(out.status, stamp_s, std::move(error));
    out.last_command.reset();
    return out;
  }

  [[nodiscard]] std::tuple<ScanRuntime, ScanStepResult> step(double stamp_s = 0.0) const
  {
    auto out = normalized();
    auto result = advance_scan_step(out.profile, out.status, stamp_s);
    out.status = result.status;
    out.last_command = result.command;
    return {out, result};
  }
};

[[nodiscard]] inline ScanRuntime make_scan_runtime(const ScanProfile & profile = ScanProfile{})
{
  ScanRuntime runtime;
  runtime.profile = profile.normalized();
  runtime.status = ScanStatus{}.normalized(runtime.profile);
  return runtime;
}

[[nodiscard]] inline std::vector<ScanStepResult> preview_scan(
  const ScanProfile & profile = ScanProfile{},
  int steps = 40,
  double start_stamp_s = 0.0)
{
  const auto item = profile.normalized();
  auto runtime = make_scan_runtime(item).start(start_stamp_s);
  std::vector<ScanStepResult> out;
  out.reserve(static_cast<std::size_t>(std::max(0, steps)));

  auto stamp = start_stamp_s;

  for (int i = 0; i < steps; ++i) {
    stamp += item.step_delay_s;
    auto [next_runtime, result] = runtime.step(stamp);
    runtime = next_runtime;
    out.push_back(result);
  }

  return out;
}

}  // namespace savo_head
