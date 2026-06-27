#ifndef SAVO_PERCEPTION__SAFETY_POLICY_HPP_
#define SAVO_PERCEPTION__SAFETY_POLICY_HPP_

#include <optional>

#include "savo_perception/constants.hpp"
#include "savo_perception/perception_types.hpp"
#include "savo_perception/range_fusion.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

struct SAVO_PERCEPTION_PUBLIC SafetyPolicyConfig
{
  RangeFusionConfig fusion{};

  int stop_debounce_count{constants::kStopDebounceCountDefault};
  int clear_debounce_count{constants::kClearDebounceCountDefault};

  double front_clear_hysteresis_m{constants::kFrontClearHysteresisMDefault};
  double side_clear_hysteresis_m{constants::kSideClearHysteresisMDefault};

  double slowdown_ema_alpha{constants::kSlowdownEmaAlphaDefault};
};

struct SAVO_PERCEPTION_PUBLIC SafetyPolicyUpdate
{
  RangeFusionResult raw_result;
  SafetyState state;
  SafetyDecision published_decision;

  int stop_count{0};
  int clear_count{0};
};

SAVO_PERCEPTION_PUBLIC int safe_debounce_count(int value);

SAVO_PERCEPTION_PUBLIC double ema_value(
  double new_value,
  double old_value,
  double alpha);

SAVO_PERCEPTION_PUBLIC RangeFusionConfig fusion_config_for_state(
  const SafetyPolicyConfig & config,
  const SafetyState & state);

class SAVO_PERCEPTION_PUBLIC SafetyPolicy
{
public:
  explicit SafetyPolicy(
    SafetyPolicyConfig config = SafetyPolicyConfig{},
    std::optional<SafetyState> initial_state = std::nullopt);

  [[nodiscard]] const SafetyPolicyConfig & config() const;
  void set_config(SafetyPolicyConfig config);

  [[nodiscard]] const SafetyState & state() const;
  [[nodiscard]] int stop_count() const;
  [[nodiscard]] int clear_count() const;
  [[nodiscard]] double slowdown_filtered() const;

  void reset();

  [[nodiscard]] SafetyPolicyUpdate update(const RangeSnapshot & snapshot);

private:
  [[nodiscard]] SafetyDecision apply_debounce(const SafetyDecision & raw_decision);
  [[nodiscard]] SafetyDecision apply_slowdown_filter(const SafetyDecision & decision);

  SafetyPolicyConfig config_;
  SafetyState state_;

  int stop_count_{0};
  int clear_count_{0};
  double slowdown_filtered_{constants::kSlowdownMaxDefault};
};

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__SAFETY_POLICY_HPP_