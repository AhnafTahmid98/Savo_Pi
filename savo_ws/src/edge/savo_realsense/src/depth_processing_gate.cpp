// Copyright 2026 Ahnaf Tahmid

#include "savo_realsense/depth_processing_gate.hpp"

#include <cmath>
#include <stdexcept>

namespace savo_realsense
{

DepthProcessingGate::DepthProcessingGate(const double max_processing_hz)
{
  if (!std::isfinite(max_processing_hz) || max_processing_hz <= 0.0) {
    throw std::invalid_argument("max_processing_hz must be finite and positive");
  }
  minimum_interval_s_ = 1.0 / max_processing_hz;
}

bool DepthProcessingGate::should_process(
  const double monotonic_time_s) noexcept
{
  if (!std::isfinite(monotonic_time_s)) {
    return false;
  }
  if (
    !have_selected_time_ || monotonic_time_s < last_selected_time_s_ ||
    monotonic_time_s - last_selected_time_s_ >= minimum_interval_s_)
  {
    last_selected_time_s_ = monotonic_time_s;
    have_selected_time_ = true;
    return true;
  }
  return false;
}

void DepthProcessingGate::reset() noexcept
{
  last_selected_time_s_ = 0.0;
  have_selected_time_ = false;
}

}  // namespace savo_realsense
