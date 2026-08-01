// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_observer/observer_contract.hpp"

#include <array>
#include <string>

#include "savo_observer/topic_contract.hpp"

namespace savo_observer
{
namespace
{

template<typename Enum, std::size_t SizeT>
std::optional<Enum> Parse(
  const std::string_view value,
  const std::array<std::pair<Enum, std::string_view>, SizeT> & entries) noexcept
{
  for (const auto & entry : entries) {
    if (entry.second == value) {
      return entry.first;
    }
  }
  return std::nullopt;
}

constexpr std::array<std::pair<ObserverState, std::string_view>, 6> kObserverStates{{
  {ObserverState::kStarting, "starting"},
  {ObserverState::kConnected, "connected"},
  {ObserverState::kDegraded, "degraded"},
  {ObserverState::kDisconnected, "disconnected"},
  {ObserverState::kFailed, "failed"},
  {ObserverState::kShuttingDown, "shutting_down"}}};

constexpr std::array<std::pair<DependencyState, std::string_view>, 6> kDependencyStates{{
  {DependencyState::kDisabled, "disabled"},
  {DependencyState::kUnavailable, "unavailable"},
  {DependencyState::kFresh, "fresh"},
  {DependencyState::kStale, "stale"},
  {DependencyState::kDegraded, "degraded"},
  {DependencyState::kFailed, "failed"}}};

}  // namespace

std::string_view ToString(const ObserverState state) noexcept
{
  for (const auto & entry : kObserverStates) {
    if (entry.first == state) {
      return entry.second;
    }
  }
  return "failed";
}

std::string_view ToString(const DependencyState state) noexcept
{
  for (const auto & entry : kDependencyStates) {
    if (entry.first == state) {
      return entry.second;
    }
  }
  return "failed";
}

std::optional<ObserverState> ParseObserverState(const std::string_view value) noexcept
{
  return Parse(value, kObserverStates);
}

std::optional<DependencyState> ParseDependencyState(const std::string_view value) noexcept
{
  return Parse(value, kDependencyStates);
}

bool IsAbsoluteTopic(const std::string_view topic) noexcept
{
  return topic.size() > 1U && topic.front() == '/' && topic.back() != '/' &&
         topic.find("//") == std::string_view::npos;
}

bool IsProhibitedInterface(const std::string_view name) noexcept
{
  for (const auto prohibited : topics::kProhibitedInterfaces) {
    if (name == prohibited ||
      (name.size() > prohibited.size() && name.substr(0, prohibited.size()) == prohibited &&
      name[prohibited.size()] == '/'))
    {
      return true;
    }
  }
  return false;
}

std::string ValidateOutputNamespace(const std::string_view output_namespace)
{
  if (!IsAbsoluteTopic(output_namespace)) {
    return "output namespace must be an absolute ROS name without a trailing slash";
  }
  if (output_namespace != "/savo_observer") {
    return "observer publications must remain under /savo_observer";
  }
  return {};
}

}  // namespace savo_observer
