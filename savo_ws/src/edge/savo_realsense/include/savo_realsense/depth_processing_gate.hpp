// Copyright 2026 Ahnaf Tahmid

#pragma once

namespace savo_realsense
{

class DepthProcessingGate
{
public:
  explicit DepthProcessingGate(double max_processing_hz);

  [[nodiscard]] bool should_process(double monotonic_time_s) noexcept;
  void reset() noexcept;

private:
  double minimum_interval_s_{0.0};
  double last_selected_time_s_{0.0};
  bool have_selected_time_{false};
};

}  // namespace savo_realsense
