// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_OBSERVER__OBSERVER_CONTRACT_HPP_
#define SAVO_OBSERVER__OBSERVER_CONTRACT_HPP_

#include <optional>
#include <string>
#include <string_view>

namespace savo_observer
{

enum class ObserverState
{
  kStarting,
  kConnected,
  kDegraded,
  kDisconnected,
  kFailed,
  kShuttingDown,
};

enum class DependencyState
{
  kDisabled,
  kUnavailable,
  kFresh,
  kStale,
  kDegraded,
  kFailed,
};

[[nodiscard]] std::string_view ToString(ObserverState state) noexcept;
[[nodiscard]] std::string_view ToString(DependencyState state) noexcept;
[[nodiscard]] std::optional<ObserverState> ParseObserverState(std::string_view value) noexcept;
[[nodiscard]] std::optional<DependencyState> ParseDependencyState(
  std::string_view value) noexcept;
[[nodiscard]] bool IsAbsoluteTopic(std::string_view topic) noexcept;
[[nodiscard]] bool IsProhibitedInterface(std::string_view name) noexcept;
[[nodiscard]] std::string ValidateOutputNamespace(std::string_view output_namespace);

}  // namespace savo_observer

#endif  // SAVO_OBSERVER__OBSERVER_CONTRACT_HPP_
