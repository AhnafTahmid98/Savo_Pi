#include "savo_vo/latest_frame_selector.hpp"

#include <cmath>

namespace savo_vo
{

bool LatestFrameSelector::offer(const double stamp_s) noexcept
{
  if (
    !std::isfinite(stamp_s) ||
    (have_seen_stamp_ && stamp_s <= newest_stamp_s_))
  {
    return false;
  }

  newest_stamp_s_ = stamp_s;
  have_seen_stamp_ = true;
  pending_ = true;
  return true;
}

std::optional<double> LatestFrameSelector::take() noexcept
{
  if (!pending_) {
    return std::nullopt;
  }

  pending_ = false;
  return newest_stamp_s_;
}

std::size_t LatestFrameSelector::pending_count() const noexcept
{
  return pending_ ? 1U : 0U;
}

bool valid_frame_interval(
  const double previous_stamp_s,
  const double current_stamp_s,
  const double max_frame_interval_s) noexcept
{
  const double interval_s = current_stamp_s - previous_stamp_s;
  return std::isfinite(interval_s) && interval_s > 0.001 &&
         interval_s <= max_frame_interval_s;
}

}  // namespace savo_vo
