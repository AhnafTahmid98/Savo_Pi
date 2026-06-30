#include "savo_perception/safety_state.hpp"

namespace savo_perception
{

static_assert(
  static_cast<int>(SafetyStatus::kOk) == 0,
  "SafetyStatus::kOk must stay stable");

static_assert(
  static_cast<int>(SafetyStatus::kSlow) == 1,
  "SafetyStatus::kSlow must stay stable");

static_assert(
  static_cast<int>(SafetyStatus::kSafetyStop) == 2,
  "SafetyStatus::kSafetyStop must stay stable");

static_assert(
  static_cast<int>(SafetyStatus::kError) == 3,
  "SafetyStatus::kError must stay stable");

}  // namespace savo_perception