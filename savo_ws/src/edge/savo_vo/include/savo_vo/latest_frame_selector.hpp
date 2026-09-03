#pragma once

#include <cstddef>
#include <optional>

namespace savo_vo
{

class LatestFrameSelector
{
public:
  [[nodiscard]] bool offer(double stamp_s) noexcept;
  [[nodiscard]] std::optional<double> take() noexcept;
  [[nodiscard]] std::size_t pending_count() const noexcept;

private:
  double newest_stamp_s_{0.0};
  bool have_seen_stamp_{false};
  bool pending_{false};
};

[[nodiscard]] bool valid_frame_interval(
  double previous_stamp_s,
  double current_stamp_s,
  double max_frame_interval_s) noexcept;

}  // namespace savo_vo
