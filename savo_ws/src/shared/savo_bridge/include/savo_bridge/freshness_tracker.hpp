// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__FRESHNESS_TRACKER_HPP_
#define SAVO_BRIDGE__FRESHNESS_TRACKER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string_view>

namespace savo_bridge
{

enum class FreshnessState : std::uint8_t
{
  kNeverObserved,
  kFresh,
  kStale,
  kClockRegression,
};

[[nodiscard]] constexpr std::string_view to_string(
  FreshnessState state) noexcept
{
  switch (state) {
    case FreshnessState::kNeverObserved:
      return "never_observed";
    case FreshnessState::kFresh:
      return "fresh";
    case FreshnessState::kStale:
      return "stale";
    case FreshnessState::kClockRegression:
      return "clock_regression";
  }

  return "unknown";
}

class FreshnessTracker final
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Duration = Clock::duration;

  struct Snapshot
  {
    FreshnessState state{FreshnessState::kNeverObserved};
    bool observed{false};
    std::uint64_t accepted_observations{0U};
    std::uint64_t rejected_regressions{0U};
    Duration age{Duration::zero()};
  };

  explicit FreshnessTracker(Duration stale_after);

  [[nodiscard]] bool observe(TimePoint observed_at) noexcept;
  [[nodiscard]] Snapshot snapshot(TimePoint evaluated_at) const noexcept;

  void reset() noexcept;

  [[nodiscard]] Duration stale_after() const noexcept;

private:
  const Duration stale_after_;

  mutable std::mutex mutex_;
  std::optional<TimePoint> last_observed_;
  std::uint64_t accepted_observations_{0U};
  std::uint64_t rejected_regressions_{0U};
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__FRESHNESS_TRACKER_HPP_
