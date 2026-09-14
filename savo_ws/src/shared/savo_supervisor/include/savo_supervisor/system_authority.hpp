// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "savo_supervisor/supervisor_state.hpp"

namespace savo_supervisor
{

enum class SystemCommand : std::uint8_t
{
  kStatus = 0U,
  kArm,
  kDisarm,
  kBeginShutdown,
  kClearFaultLatch,
};

enum class SystemAuthorityState : std::uint8_t
{
  kBooting = 0U,
  kReadyToArm,
  kArmed,
  kArmedDegraded,
  kDisarmed,
  kFaultLatched,
  kShutdownRequested,
};

enum class SystemAuthorityCode : std::uint8_t
{
  kAccepted = 0U,
  kInvalidRequest,
  kNotReady,
  kAlreadyInState,
  kFaultLatched,
  kUnsafeToClear,
  kGenerationMismatch,
};

enum class CoreFaultKind
{
  kNone,
  kUnavailable,
  kCritical,
};

struct CoreFaultEvidence
{
  CoreFaultKind kind{CoreFaultKind::kNone};
  std::string reason;
};

struct SystemDependencySnapshot
{
  bool core_ready{false};
  CoreFaultEvidence core_fault;
  bool safety_known{false};
  bool startup_dependencies_ready{false};
  bool degraded{false};
  bool remote_commands_ready{false};
  bool mission_idle{true};
};

[[nodiscard]] SystemDependencySnapshot EvaluateCoreSystemDependencies(const SupervisorState & core);

struct SystemAuthorityPolicy
{
  bool auto_arm{false};
  bool latch_core_faults_after_arm{true};
};

struct SystemAuthorityRequest
{
  SystemCommand command{SystemCommand::kStatus};
  std::string request_id;
  std::string actor_id;
  std::string reason;
  std::uint64_t expected_generation{0U};
};

struct SystemAuthorityDecision
{
  bool accepted{false};
  SystemAuthorityCode code{SystemAuthorityCode::kInvalidRequest};
  std::string reason{"not_evaluated"};
};

struct SystemAuthoritySnapshot
{
  SystemAuthorityState state{SystemAuthorityState::kBooting};
  bool startup_ready{false};
  bool armed{false};
  bool fault_latched{false};
  bool shutdown_requested{false};
  bool remote_commands_ready{false};
  std::uint64_t generation{0U};
  std::string reason{"system_booting"};
  std::string last_actor;
};

class SystemAuthority
{
public:
  explicit SystemAuthority(SystemAuthorityPolicy policy = {});

  [[nodiscard]] bool Update(
    const SystemDependencySnapshot & dependencies,
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

  [[nodiscard]] SystemAuthorityDecision Handle(
    const SystemAuthorityRequest & request,
    const SystemDependencySnapshot & dependencies);

  void RestoreFaultLatch(
    bool fault_latched,
    std::uint64_t generation,
    const std::string & reason);

  [[nodiscard]] SystemAuthoritySnapshot snapshot(
    const SystemDependencySnapshot & dependencies) const;

private:
  void change(std::string reason, std::string actor = {});

  SystemAuthorityPolicy policy_{};
  bool armed_{false};
  bool fault_latched_{false};
  bool shutdown_requested_{false};
  bool rearm_required_{false};
  std::optional<std::chrono::steady_clock::time_point> unavailable_since_;
  std::uint64_t generation_{0U};
  std::string reason_{"system_booting"};
  std::string last_actor_{};
};

[[nodiscard]] const char * ToString(SystemAuthorityState value) noexcept;

}  // namespace savo_supervisor
