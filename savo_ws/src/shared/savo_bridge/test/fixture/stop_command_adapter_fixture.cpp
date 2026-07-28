// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include "savo_bridge/command_protocol.hpp"
#include "savo_bridge/command_server.hpp"
#include "savo_bridge/stop_command_adapter.hpp"
#include "support/recording_stop_adapter.hpp"

namespace
{

using Json = nlohmann::json;

[[nodiscard]] std::int64_t now_unix_ms() noexcept
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

[[nodiscard]] Json result_document(
  const std::optional<std::string> & command_id,
  const savo_bridge::StopAdapterResult & result,
  const std::size_t invocation_count)
{
  return Json{
    {"protocol_version", 1},
    {"message_type", "stop_adapter_test_result"},
    {"command_id", command_id.value_or("")},
    {
      "adapter_invocation_attempted",
      result.adapter_invocation_attempted
    },
    {"adapter_invoked", result.adapter_invoked},
    {"adapter_accepted", result.adapter_accepted},
    {"state", savo_bridge::to_string(result.state)},
    {"reason", result.reason},
    {
      "physical_dispatch_attempted",
      result.physical_dispatch_attempted
    },
    {"physical_effect", result.physical_effect},
    {
      "physical_stop_confirmed",
      result.physical_stop_confirmed
    },
    {"adapter_invocation_count", invocation_count},
    {"ros_publications", 0},
  };
}

[[nodiscard]] savo_bridge::StopAdapterResult invalid_request_result(
  const std::string & reason)
{
  savo_bridge::StopAdapterResult result;
  result.state = savo_bridge::StopAdapterState::Rejected;
  result.reason = reason;
  return result;
}

}  // namespace

int main()
{
  std::string encoded_request;
  if (!std::getline(std::cin, encoded_request) ||
    std::cin.eof() ||
    encoded_request.empty() ||
    encoded_request.size() >
    savo_bridge::DEFAULT_COMMAND_MAX_REQUEST_BYTES)
  {
    std::cerr << "invalid newline-terminated command request\n";
    return 2;
  }

  if (std::cin.peek() != std::char_traits<char>::eof()) {
    std::cerr << "fixture accepts exactly one command request\n";
    return 2;
  }

  const auto parsed =
    savo_bridge::parse_command_request(
    encoded_request,
    now_unix_ms());

  if (!parsed.succeeded() || !parsed.command.has_value()) {
    const auto result =
      invalid_request_result(
      "native_stop_adapter_request_invalid");
    std::cout << result_document(
      std::nullopt,
      result,
      0U).dump() << '\n';
    return 0;
  }

  auto adapter =
    std::make_shared<
    savo_bridge::test::RecordingStopAdapter>();
  const savo_bridge::StopAdapterController controller{
    savo_bridge::StopAdapterControllerConfig{true},
    adapter,
  };
  const auto result =
    controller.request_stop(parsed.command.value());

  std::cout << result_document(
    parsed.command->command_id,
    result,
    adapter->invocation_count()).dump() << '\n';
  return 0;
}
