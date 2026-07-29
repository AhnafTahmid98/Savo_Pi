// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace savo_nav
{

enum class CoverageExecutionState : std::uint8_t
{
  kIdle = 0,
  kWaitingForBackend,
  kExecuting,
  kCanceling,
  kSucceeded,
  kRejected,
  kFailed,
  kCanceled,
  kTimedOut,
  kFeedbackStale
};

enum class CoverageTerminalCode : std::uint8_t
{
  kNone = 0,
  kSucceeded,
  kInvalidRequest,
  kNotReady,
  kBusy,
  kBackendUnavailable,
  kBackendRejected,
  kAborted,
  kCanceled,
  kTimedOut,
  kFeedbackStale,
  kInternalError
};

enum class CoverageBackendTerminal : std::uint8_t
{
  kSucceeded = 0,
  kRejected,
  kAborted,
  kCanceled,
  kUnknown
};

enum class CoverageWatchdogEvent : std::uint8_t
{
  kNone = 0,
  kExecutionTimeout,
  kFeedbackStale,
  kCancelTimeout
};

struct CoverageExecutionPolicy
{
  double default_execution_timeout_seconds{300.0};
  double maximum_execution_timeout_seconds{3600.0};
  double feedback_stale_timeout_seconds{10.0};
  double cancel_timeout_seconds{5.0};
};

struct CoverageExecutionSnapshot
{
  CoverageExecutionState state{
    CoverageExecutionState::kIdle};

  CoverageTerminalCode terminal_code{
    CoverageTerminalCode::kNone};

  std::string reason{"idle"};

  double resolved_execution_timeout_seconds{0.0};

  bool cancel_requested{false};
  bool public_terminal{false};
  bool release_ownership{false};
  bool backend_terminal_pending{false};
};

class CoverageExecutionModel
{
public:
  explicit CoverageExecutionModel(
    CoverageExecutionPolicy policy = {});

  [[nodiscard]] bool Start(
    double now_seconds,
    double requested_timeout_seconds);

  [[nodiscard]] bool MarkBackendAccepted(
    double now_seconds);

  [[nodiscard]] bool MarkFeedback(
    double now_seconds);

  [[nodiscard]] bool RequestCancel(
    double now_seconds,
    std::string reason);

  [[nodiscard]] CoverageWatchdogEvent CheckWatchdogs(
    double now_seconds);

  [[nodiscard]] bool FailBeforeBackend(
    CoverageTerminalCode code,
    std::string reason);

  [[nodiscard]] bool ObserveBackendTerminal(
    CoverageBackendTerminal terminal,
    std::string reason);

  void Reset() noexcept;

  [[nodiscard]] const CoverageExecutionSnapshot &
  Snapshot() const noexcept;

  [[nodiscard]] static std::string_view ToString(
    CoverageExecutionState state) noexcept;

  [[nodiscard]] static std::string_view ToString(
    CoverageTerminalCode code) noexcept;

private:
  [[nodiscard]] bool IsPolicyValid() const noexcept;
  [[nodiscard]] bool IsTimeValid(double value) const noexcept;

  void SetPublicTerminal(
    CoverageExecutionState state,
    CoverageTerminalCode code,
    std::string reason,
    bool release_ownership);

  CoverageExecutionPolicy policy_{};
  CoverageExecutionSnapshot snapshot_{};

  double started_at_seconds_{0.0};
  double backend_accepted_at_seconds_{0.0};
  double last_feedback_seconds_{0.0};
  double cancel_started_at_seconds_{0.0};
  double last_event_seconds_{0.0};

  std::string cancellation_reason_{};
  bool backend_accepted_{false};
};

}  // namespace savo_nav
