// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace savo_nav
{

enum class ObservedControlMode : std::uint8_t
{
  kUnknown = 0,
  kStop,
  kManual,
  kAuto,
  kNav,
  kRecovery
};

struct ControlModeObservation
{
  bool observed{false};
  bool fresh{false};

  bool navigation_allowed{false};
  bool cancel_active_goal{true};

  ObservedControlMode mode{
    ObservedControlMode::kUnknown};

  double age_seconds{0.0};

  std::string mode_text{"UNKNOWN"};
  std::string reason_text{};
  std::string status_text{};

  std::string reason_code{
    "control_mode_unobserved"};
};

class ControlModeClient
{
public:
  [[nodiscard]] bool UpdateMode(
    std::string mode_text,
    double now_seconds);

  void UpdateReason(std::string reason_text);

  void UpdateStatus(std::string status_text);

  void Reset() noexcept;

  [[nodiscard]] ControlModeObservation Evaluate(
    double now_seconds,
    double timeout_seconds) const;

  [[nodiscard]] static ObservedControlMode Parse(
    std::string_view mode_text);

  [[nodiscard]] static std::string_view ToString(
    ObservedControlMode mode) noexcept;

private:
  bool observed_{false};

  double last_mode_seconds_{0.0};

  ObservedControlMode mode_{
    ObservedControlMode::kUnknown};

  std::string mode_text_{"UNKNOWN"};
  std::string reason_text_{};
  std::string status_text_{};
};

}  // namespace savo_nav
