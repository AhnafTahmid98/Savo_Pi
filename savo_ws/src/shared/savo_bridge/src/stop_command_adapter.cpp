// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/stop_command_adapter.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace savo_bridge
{
namespace
{

constexpr std::string_view DISABLED_REASON{
  "native_stop_adapter_disabled"};
constexpr std::string_view MISSING_REASON{
  "native_stop_adapter_missing"};
constexpr std::string_view INVALID_REQUEST_REASON{
  "native_stop_adapter_request_invalid"};
constexpr std::string_view REJECTED_REASON{
  "native_stop_adapter_rejected"};
constexpr std::string_view CONTRACT_VIOLATION_REASON{
  "native_stop_adapter_contract_violation"};
constexpr std::string_view INTERNAL_FAILURE_REASON{
  "native_stop_adapter_internal_failure"};
constexpr std::string_view UNSUPPORTED_REASON{
  "native_stop_adapter_unsupported_command"};
constexpr std::string_view TEST_ACCEPTED_REASON{
  "native_stop_test_adapter_accepted"};

[[nodiscard]] StopAdapterResult result_with(
  const StopAdapterState state,
  const std::string_view reason,
  const bool attempted = false,
  const bool invoked = false) noexcept
{
  StopAdapterResult result;
  result.adapter_invocation_attempted = attempted;
  result.adapter_invoked = invoked;
  result.state = state;
  result.reason = std::string(reason);
  return result;
}

[[nodiscard]] bool is_ascii_alphanumeric(
  const char character) noexcept
{
  return
    (character >= 'A' && character <= 'Z') ||
    (character >= 'a' && character <= 'z') ||
    (character >= '0' && character <= '9');
}

[[nodiscard]] bool is_safe_identifier(
  const std::string_view value) noexcept
{
  if (value.empty() || value.size() > 128U ||
    !is_ascii_alphanumeric(value.front()))
  {
    return false;
  }

  return std::all_of(
    value.begin() + 1,
    value.end(),
    [](const char character) {
      return
        is_ascii_alphanumeric(character) ||
        character == '.' ||
        character == '_' ||
        character == ':' ||
        character == '-';
    });
}

[[nodiscard]] bool is_valid_source(
  const std::string_view value) noexcept
{
  if (value.empty() || value.size() > 64U ||
    value.front() < 'a' || value.front() > 'z')
  {
    return false;
  }

  return std::all_of(
    value.begin() + 1,
    value.end(),
    [](const char character) {
      return
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9') ||
        character == '.' ||
        character == '_' ||
        character == ':' ||
        character == '-';
    });
}

[[nodiscard]] bool is_valid_origin_agent(
  const std::string_view value) noexcept
{
  if (value.empty() || value.size() > 64U ||
    value.front() < 'a' || value.front() > 'z')
  {
    return false;
  }

  return std::all_of(
    value.begin() + 1,
    value.end(),
    [](const char character) {
      return
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9') ||
        character == '_';
    });
}

[[nodiscard]] bool is_valid_reason(
  const std::string_view value) noexcept
{
  if (value.empty() || value.size() > 128U) {
    return false;
  }

  const auto is_whitespace = [](const char character) {
      return
        character == ' ' ||
        character == '\t' ||
        character == '\n' ||
        character == '\r' ||
        character == '\f' ||
        character == '\v';
    };

  return std::any_of(
    value.begin(),
    value.end(),
    [is_whitespace](const char character) {
      return !is_whitespace(character);
    });
}

[[nodiscard]] bool valid_stop_semantics(
  const ValidatedCommand & command,
  const StopCommandPayload & payload) noexcept
{
  const bool origin_valid =
    !command.origin_agent.has_value() ||
    is_valid_origin_agent(command.origin_agent.value());

  return
    command.protocol_version == COMMAND_PROTOCOL_VERSION &&
    command.message_type == "command_request" &&
    is_safe_identifier(command.command_id) &&
    is_valid_source(command.source) &&
    origin_valid &&
    command.priority == CommandPriority::Emergency &&
    is_valid_reason(payload.reason) &&
    payload.scope == "all_movement" &&
    command.issued_at_unix_ms >= 0 &&
    command.expires_at_unix_ms > command.issued_at_unix_ms &&
    command.expires_at_unix_ms - command.issued_at_unix_ms <=
    MAX_COMMAND_TTL_MS;
}

[[nodiscard]] StopAdapterResult contract_violation(
  const std::string & command_id) noexcept
{
  auto result = result_with(
    StopAdapterState::Rejected,
    CONTRACT_VIOLATION_REASON,
    true,
    true);
  result.command_id = command_id;
  return result;
}

}  // namespace

std::string_view to_string(
  const StopAdapterState state) noexcept
{
  switch (state) {
    case StopAdapterState::Disabled:
      return "disabled";
    case StopAdapterState::TestNoEffect:
      return "test_no_effect";
    case StopAdapterState::Rejected:
      return "rejected";
  }

  return "rejected";
}

StopAdapterResult StopCommandAdapter::request_stop(
  const StopAdapterRequest & request) noexcept
{
  try {
    return request_stop_impl(request);
  } catch (...) {
    auto result = result_with(
      StopAdapterState::Rejected,
      INTERNAL_FAILURE_REASON,
      true,
      true);
    result.command_id = request.command_id;
    return result;
  }
}

StopAdapterController::StopAdapterController(
  StopAdapterControllerConfig config,
  std::shared_ptr<StopCommandAdapter> adapter) noexcept
: config_(config),
  adapter_(std::move(adapter))
{
}

StopAdapterResult StopAdapterController::request_stop(
  const ValidatedCommand & command) const noexcept
{
  if (!config_.enabled) {
    return result_with(
      StopAdapterState::Disabled,
      DISABLED_REASON);
  }

  if (command.command_type != CommandType::Stop) {
    return result_with(
      StopAdapterState::Rejected,
      UNSUPPORTED_REASON);
  }

  const auto * payload =
    std::get_if<StopCommandPayload>(&command.payload);
  if (payload == nullptr ||
    !valid_stop_semantics(command, *payload))
  {
    return result_with(
      StopAdapterState::Rejected,
      INVALID_REQUEST_REASON);
  }

  if (!adapter_) {
    auto result = result_with(
      StopAdapterState::Rejected,
      MISSING_REASON);
    result.command_id = command.command_id;
    return result;
  }

  const StopAdapterRequest request{
    command.command_id,
    command.source,
    command.origin_agent,
    command.priority,
    payload->reason,
    payload->scope,
    command.issued_at_unix_ms,
    command.expires_at_unix_ms,
  };

  StopAdapterResult adapter_result =
    adapter_->request_stop(request);

  if (adapter_result.physical_dispatch_attempted ||
    adapter_result.physical_effect ||
    adapter_result.physical_stop_confirmed)
  {
    return contract_violation(command.command_id);
  }

  if (!adapter_result.adapter_invocation_attempted ||
    !adapter_result.adapter_invoked)
  {
    return contract_violation(command.command_id);
  }

  if (adapter_result.command_id.has_value() &&
    adapter_result.command_id.value() != command.command_id)
  {
    return contract_violation(command.command_id);
  }

  if (adapter_result.adapter_name.has_value() &&
    !is_safe_identifier(adapter_result.adapter_name.value()))
  {
    return contract_violation(command.command_id);
  }

  if (!adapter_result.adapter_accepted) {
    if (adapter_result.state != StopAdapterState::Rejected ||
      adapter_result.reason == TEST_ACCEPTED_REASON)
    {
      return contract_violation(command.command_id);
    }

    auto result = result_with(
      StopAdapterState::Rejected,
      adapter_result.reason == INTERNAL_FAILURE_REASON ?
      INTERNAL_FAILURE_REASON :
      REJECTED_REASON,
      true,
      true);
    result.command_id = command.command_id;
    result.adapter_name = adapter_result.adapter_name;
    return result;
  }

  if (adapter_result.state != StopAdapterState::TestNoEffect ||
    adapter_result.reason != TEST_ACCEPTED_REASON)
  {
    return contract_violation(command.command_id);
  }

  adapter_result.command_id = command.command_id;
  return adapter_result;
}

}  // namespace savo_bridge
