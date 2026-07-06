#include "savo_perception/safety_policy.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace savo_perception
{

int safe_debounce_count(const int value)
{
  return std::max(1, value);
}

double ema_value(
  const double new_value,
  const double old_value,
  const double alpha)
{
  const auto a = clamp_value(alpha, 0.0, 1.0);
  return (a * new_value) + ((1.0 - a) * old_value);
}

RangeFusionConfig fusion_config_for_state(
  const SafetyPolicyConfig & config,
  const SafetyState & state)
{
  auto fusion = config.fusion;

  if (state.stop_required()) {
    fusion.front_stop_m += std::max(0.0, config.front_clear_hysteresis_m);
    fusion.side_stop_m += std::max(0.0, config.side_clear_hysteresis_m);
  }

  return fusion;
}

SafetyPolicy::SafetyPolicy(
  SafetyPolicyConfig config,
  std::optional<SafetyState> initial_state)
: config_(std::move(config)),
  state_(initial_state.value_or(make_initial_safety_state()))
{
  stop_count_ = state_.stop_count;
  clear_count_ = state_.clear_count;
  slowdown_filtered_ = clamp_slowdown(
    state_.active_decision.slowdown_factor,
    config_.fusion.slowdown_min,
    config_.fusion.slowdown_max);
}

const SafetyPolicyConfig & SafetyPolicy::config() const
{
  return config_;
}

void SafetyPolicy::set_config(SafetyPolicyConfig config)
{
  config_ = std::move(config);
  slowdown_filtered_ = clamp_slowdown(
    slowdown_filtered_,
    config_.fusion.slowdown_min,
    config_.fusion.slowdown_max);
}

const SafetyState & SafetyPolicy::state() const
{
  return state_;
}

int SafetyPolicy::stop_count() const
{
  return stop_count_;
}

int SafetyPolicy::clear_count() const
{
  return clear_count_;
}

double SafetyPolicy::slowdown_filtered() const
{
  return slowdown_filtered_;
}

void SafetyPolicy::reset()
{
  state_ = make_initial_safety_state();
  stop_count_ = 0;
  clear_count_ = 0;
  slowdown_filtered_ = constants::kSlowdownMaxDefault;
}

SafetyPolicyUpdate SafetyPolicy::update(const RangeSnapshot & snapshot)
{
  const auto fusion_config = fusion_config_for_state(config_, state_);
  auto raw_result = fuse_range_snapshot(snapshot, fusion_config);

  auto debounced_decision = apply_debounce(raw_result.decision);
  auto published_decision = apply_slowdown_filter(debounced_decision);

  state_ = with_decision(
    state_,
    published_decision,
    stop_count_,
    clear_count_);

  SafetyPolicyUpdate update;
  update.raw_result = std::move(raw_result);
  update.state = state_;
  update.published_decision = published_decision;
  update.stop_count = stop_count_;
  update.clear_count = clear_count_;

  return update;
}

SafetyDecision SafetyPolicy::apply_debounce(const SafetyDecision & raw_decision)
{
  const auto stop_threshold = safe_debounce_count(config_.stop_debounce_count);
  const auto clear_threshold = safe_debounce_count(config_.clear_debounce_count);

  if (raw_decision.stop_required) {
    stop_count_ = std::min(stop_count_ + 1, stop_threshold);
    clear_count_ = 0;

    if (state_.stop_required() || stop_count_ >= stop_threshold) {
      return raw_decision;
    }

    auto held = state_.active_decision;
    held.reason = "stop_debounce_pending";
    held.stale_sensors = raw_decision.stale_sensors;
    held.invalid_sensors = raw_decision.invalid_sensors;
    held.front_distance_m = raw_decision.front_distance_m;
    held.side_distance_m = raw_decision.side_distance_m;
    return held;
  }

  clear_count_ = std::min(clear_count_ + 1, clear_threshold);
  stop_count_ = 0;

  if (state_.stop_required() && clear_count_ < clear_threshold) {
    auto held = state_.active_decision;
    held.reason = "clear_debounce_pending";
    held.front_distance_m = raw_decision.front_distance_m;
    held.side_distance_m = raw_decision.side_distance_m;
    held.stale_sensors = raw_decision.stale_sensors;
    held.invalid_sensors = raw_decision.invalid_sensors;
    return held;
  }

  return raw_decision;
}

SafetyDecision SafetyPolicy::apply_slowdown_filter(const SafetyDecision & decision)
{
  auto out = decision;

  if (out.stop_required) {
    slowdown_filtered_ = 0.0;
    out.slowdown_factor = 0.0;
    return out;
  }

  const auto target = clamp_slowdown(
    out.slowdown_factor,
    config_.fusion.slowdown_min,
    config_.fusion.slowdown_max);

  slowdown_filtered_ = ema_value(
    target,
    slowdown_filtered_,
    config_.slowdown_ema_alpha);

  slowdown_filtered_ = clamp_slowdown(
    slowdown_filtered_,
    config_.fusion.slowdown_min,
    config_.fusion.slowdown_max);

  out.slowdown_factor = slowdown_filtered_;
  return out;
}

}  // namespace savo_perception
