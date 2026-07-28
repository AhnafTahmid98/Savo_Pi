// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__STOP_COMMAND_ADAPTER_HPP_
#define SAVO_BRIDGE__STOP_COMMAND_ADAPTER_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "savo_bridge/command_protocol.hpp"

namespace savo_bridge
{

struct StopAdapterRequest
{
  std::string command_id;
  std::string source;
  std::optional<std::string> origin_agent;
  CommandPriority priority{CommandPriority::Emergency};
  std::string reason;
  std::string scope;
  std::int64_t issued_at_unix_ms{0};
  std::int64_t expires_at_unix_ms{0};
};

enum class StopAdapterState
{
  Disabled,
  TestNoEffect,
  Rejected,
};

[[nodiscard]] std::string_view to_string(
  StopAdapterState state) noexcept;

struct StopAdapterResult
{
  bool adapter_invocation_attempted{false};
  bool adapter_invoked{false};
  bool adapter_accepted{false};
  bool physical_dispatch_attempted{false};
  bool physical_effect{false};
  bool physical_stop_confirmed{false};
  StopAdapterState state{StopAdapterState::Rejected};
  std::string reason{"native_stop_adapter_rejected"};
  std::optional<std::string> adapter_name;
  std::optional<std::string> command_id;
};

class StopCommandAdapter
{
public:
  virtual ~StopCommandAdapter() = default;

  [[nodiscard]] StopAdapterResult request_stop(
    const StopAdapterRequest & request) noexcept;

protected:
  [[nodiscard]] virtual StopAdapterResult request_stop_impl(
    const StopAdapterRequest & request) = 0;
};

struct StopAdapterControllerConfig
{
  bool enabled{false};
};

class StopAdapterController
{
public:
  explicit StopAdapterController(
    StopAdapterControllerConfig config = {},
    std::shared_ptr<StopCommandAdapter> adapter = nullptr) noexcept;

  [[nodiscard]] StopAdapterResult request_stop(
    const ValidatedCommand & command) const noexcept;

private:
  StopAdapterControllerConfig config_;
  std::shared_ptr<StopCommandAdapter> adapter_;
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__STOP_COMMAND_ADAPTER_HPP_
