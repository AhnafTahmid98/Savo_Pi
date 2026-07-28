// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "savo_bridge/command_protocol.hpp"
#include "savo_bridge/stop_command_adapter.hpp"
#include "support/recording_stop_adapter.hpp"

namespace
{

using savo_bridge::CommandPriority;
using savo_bridge::CommandType;
using savo_bridge::StopAdapterController;
using savo_bridge::StopAdapterControllerConfig;
using savo_bridge::StopAdapterRequest;
using savo_bridge::StopAdapterResult;
using savo_bridge::StopAdapterState;
using savo_bridge::StopCommandAdapter;
using savo_bridge::StopCommandPayload;
using savo_bridge::ValidatedCommand;
using savo_bridge::test::RecordingStopAdapter;

[[nodiscard]] ValidatedCommand valid_stop()
{
  ValidatedCommand command;
  command.command_id = "adapter-stop-001";
  command.command_type = CommandType::Stop;
  command.source = "savomind";
  command.origin_agent = "safety_agent";
  command.priority = CommandPriority::Emergency;
  command.issued_at_unix_ms = 1800000000000;
  command.expires_at_unix_ms = 1800000005000;
  command.payload = StopCommandPayload{
    "savomind_deterministic_stop",
    "all_movement",
  };
  return command;
}

void expect_no_physical_effect(
  const StopAdapterResult & result)
{
  EXPECT_FALSE(result.physical_dispatch_attempted);
  EXPECT_FALSE(result.physical_effect);
  EXPECT_FALSE(result.physical_stop_confirmed);
}

class ConfigurableAdapter final : public StopCommandAdapter
{
public:
  explicit ConfigurableAdapter(StopAdapterResult result)
  : result_(std::move(result))
  {
  }

  [[nodiscard]] std::size_t invocation_count() const noexcept
  {
    return invocation_count_;
  }

protected:
  [[nodiscard]] StopAdapterResult request_stop_impl(
    const StopAdapterRequest & request) override
  {
    ++invocation_count_;
    last_command_id_ = request.command_id;
    return result_;
  }

private:
  StopAdapterResult result_;
  std::size_t invocation_count_{0U};
  std::string last_command_id_;
};

class ThrowingAdapter final : public StopCommandAdapter
{
public:
  [[nodiscard]] std::size_t invocation_count() const noexcept
  {
    return invocation_count_;
  }

protected:
  [[nodiscard]] StopAdapterResult request_stop_impl(
    const StopAdapterRequest &) override
  {
    ++invocation_count_;
    throw std::runtime_error(
            "arbitrary adapter exception text");
  }

private:
  std::size_t invocation_count_{0U};
};

[[nodiscard]] StopAdapterResult accepted_result()
{
  StopAdapterResult result;
  result.adapter_invocation_attempted = true;
  result.adapter_invoked = true;
  result.adapter_accepted = true;
  result.state = StopAdapterState::TestNoEffect;
  result.reason = "native_stop_test_adapter_accepted";
  result.adapter_name = "test_adapter";
  return result;
}

[[nodiscard]] StopAdapterResult rejected_result()
{
  StopAdapterResult result;
  result.adapter_invocation_attempted = true;
  result.adapter_invoked = true;
  result.state = StopAdapterState::Rejected;
  result.reason = "adapter-specific rejection detail";
  return result;
}

TEST(StopAdapterControllerTest, DefaultsToDisabled)
{
  const StopAdapterController controller;
  const auto result = controller.request_stop(valid_stop());

  EXPECT_FALSE(result.adapter_invocation_attempted);
  EXPECT_FALSE(result.adapter_invoked);
  EXPECT_FALSE(result.adapter_accepted);
  EXPECT_EQ(result.state, StopAdapterState::Disabled);
  EXPECT_EQ(result.reason, "native_stop_adapter_disabled");
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, DisabledControllerNeverInvokesAdapter)
{
  auto adapter = std::make_shared<RecordingStopAdapter>();
  const StopAdapterController controller{
    StopAdapterControllerConfig{false},
    adapter,
  };

  const auto result = controller.request_stop(valid_stop());

  EXPECT_EQ(adapter->invocation_count(), 0U);
  EXPECT_FALSE(result.adapter_invocation_attempted);
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, EnabledControllerRejectsMissingAdapter)
{
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    nullptr,
  };

  const auto result = controller.request_stop(valid_stop());

  EXPECT_EQ(result.reason, "native_stop_adapter_missing");
  EXPECT_FALSE(result.adapter_invocation_attempted);
  EXPECT_FALSE(result.adapter_invoked);
  EXPECT_FALSE(result.adapter_accepted);
  EXPECT_EQ(result.state, StopAdapterState::Rejected);
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, ValidatedStopIsRecordedExactlyOnce)
{
  auto adapter = std::make_shared<RecordingStopAdapter>();
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };
  const auto command = valid_stop();

  const auto result = controller.request_stop(command);

  ASSERT_EQ(adapter->invocation_count(), 1U);
  ASSERT_TRUE(adapter->last_request().has_value());
  const auto & request = adapter->last_request().value();
  EXPECT_EQ(request.command_id, command.command_id);
  EXPECT_EQ(request.source, command.source);
  EXPECT_EQ(request.origin_agent, command.origin_agent);
  EXPECT_EQ(request.priority, CommandPriority::Emergency);
  EXPECT_EQ(request.reason, "savomind_deterministic_stop");
  EXPECT_EQ(request.scope, "all_movement");
  EXPECT_EQ(
    request.issued_at_unix_ms,
    command.issued_at_unix_ms);
  EXPECT_EQ(
    request.expires_at_unix_ms,
    command.expires_at_unix_ms);

  EXPECT_TRUE(result.adapter_invocation_attempted);
  EXPECT_TRUE(result.adapter_invoked);
  EXPECT_TRUE(result.adapter_accepted);
  EXPECT_EQ(result.state, StopAdapterState::TestNoEffect);
  EXPECT_EQ(
    savo_bridge::to_string(result.state),
    "test_no_effect");
  EXPECT_EQ(
    result.reason,
    "native_stop_test_adapter_accepted");
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, AdapterRejectionIsStableAndNotRetried)
{
  auto adapter =
    std::make_shared<ConfigurableAdapter>(
    rejected_result());
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };

  const auto result = controller.request_stop(valid_stop());

  EXPECT_EQ(adapter->invocation_count(), 1U);
  EXPECT_TRUE(result.adapter_invocation_attempted);
  EXPECT_TRUE(result.adapter_invoked);
  EXPECT_FALSE(result.adapter_accepted);
  EXPECT_EQ(result.state, StopAdapterState::Rejected);
  EXPECT_EQ(result.reason, "native_stop_adapter_rejected");
  EXPECT_EQ(
    result.reason.find("adapter-specific"),
    std::string::npos);
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, AdapterExceptionIsContainedWithoutRetry)
{
  auto adapter = std::make_shared<ThrowingAdapter>();
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };

  const auto result = controller.request_stop(valid_stop());

  EXPECT_EQ(adapter->invocation_count(), 1U);
  EXPECT_FALSE(result.adapter_accepted);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_internal_failure");
  EXPECT_EQ(
    result.reason.find("arbitrary"),
    std::string::npos);
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, UnsupportedCommandsNeverReachAdapter)
{
  const std::vector<CommandType> unsupported{
    CommandType::TeleopNudge,
    CommandType::NavigateToLocation,
    CommandType::CancelNavigation,
    CommandType::CancelAction,
  };

  for (const auto command_type : unsupported) {
    auto adapter =
      std::make_shared<RecordingStopAdapter>();
    const StopAdapterController controller{
      StopAdapterControllerConfig{true},
      adapter,
    };
    auto command = valid_stop();
    command.command_type = command_type;

    const auto result = controller.request_stop(command);

    EXPECT_EQ(adapter->invocation_count(), 0U);
    EXPECT_FALSE(result.adapter_invocation_attempted);
    EXPECT_FALSE(result.adapter_accepted);
    EXPECT_EQ(
      result.reason,
      "native_stop_adapter_unsupported_command");
    expect_no_physical_effect(result);
  }
}

TEST(StopAdapterControllerTest, NonEmergencyStopNeverReachesAdapter)
{
  auto adapter = std::make_shared<RecordingStopAdapter>();
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };
  auto command = valid_stop();
  command.priority = CommandPriority::High;

  const auto result = controller.request_stop(command);

  EXPECT_EQ(adapter->invocation_count(), 0U);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_request_invalid");
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, InvalidStopScopeNeverReachesAdapter)
{
  auto adapter = std::make_shared<RecordingStopAdapter>();
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };
  auto command = valid_stop();
  command.payload = StopCommandPayload{
    "savomind_deterministic_stop",
    "partial_movement",
  };

  const auto result = controller.request_stop(command);

  EXPECT_EQ(adapter->invocation_count(), 0U);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_request_invalid");
}

TEST(StopAdapterControllerTest, InvalidCommandIdNeverReachesAdapter)
{
  auto adapter = std::make_shared<RecordingStopAdapter>();
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };
  auto command = valid_stop();
  command.command_id = "unsafe command id";

  const auto result = controller.request_stop(command);

  EXPECT_EQ(adapter->invocation_count(), 0U);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_request_invalid");
}

TEST(StopAdapterControllerTest, InvalidTimingNeverReachesAdapter)
{
  const std::vector<std::pair<std::int64_t, std::int64_t>>
  invalid_timing{
    {-1, 100},
    {100, 100},
    {101, 100},
    {100, 60101},
  };

  for (const auto & timing : invalid_timing) {
    auto adapter =
      std::make_shared<RecordingStopAdapter>();
    const StopAdapterController controller{
      StopAdapterControllerConfig{true},
      adapter,
    };
    auto command = valid_stop();
    command.issued_at_unix_ms = timing.first;
    command.expires_at_unix_ms = timing.second;

    const auto result = controller.request_stop(command);

    EXPECT_EQ(adapter->invocation_count(), 0U);
    EXPECT_EQ(
      result.reason,
      "native_stop_adapter_request_invalid");
  }
}

TEST(StopAdapterControllerTest, MalformedStopFieldsFailClosed)
{
  std::vector<ValidatedCommand> invalid;

  auto bad_protocol = valid_stop();
  bad_protocol.protocol_version = 2;
  invalid.push_back(bad_protocol);

  auto bad_message = valid_stop();
  bad_message.message_type = "command_result";
  invalid.push_back(bad_message);

  auto bad_source = valid_stop();
  bad_source.source = "Bad Source";
  invalid.push_back(bad_source);

  auto bad_origin = valid_stop();
  bad_origin.origin_agent = "bad origin";
  invalid.push_back(bad_origin);

  auto bad_reason = valid_stop();
  bad_reason.payload = StopCommandPayload{
    "   ",
    "all_movement",
  };
  invalid.push_back(bad_reason);

  for (const auto & command : invalid) {
    auto adapter =
      std::make_shared<RecordingStopAdapter>();
    const StopAdapterController controller{
      StopAdapterControllerConfig{true},
      adapter,
    };

    const auto result = controller.request_stop(command);

    EXPECT_EQ(adapter->invocation_count(), 0U);
    EXPECT_EQ(
      result.reason,
      "native_stop_adapter_request_invalid");
  }
}

TEST(StopAdapterControllerTest, WrongPayloadVariantFailsClosed)
{
  auto adapter = std::make_shared<RecordingStopAdapter>();
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };
  auto command = valid_stop();
  command.payload =
    savo_bridge::CancelActionCommandPayload{
    "adapter-stop-001",
    "cancel",
  };

  const auto result = controller.request_stop(command);

  EXPECT_EQ(adapter->invocation_count(), 0U);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_request_invalid");
}

TEST(StopAdapterControllerTest, PhysicalClaimsAreRejectedAndCleared)
{
  for (const int field : {0, 1, 2}) {
    auto adapter_result = accepted_result();
    adapter_result.physical_dispatch_attempted =
      field == 0;
    adapter_result.physical_effect = field == 1;
    adapter_result.physical_stop_confirmed =
      field == 2;
    auto adapter =
      std::make_shared<ConfigurableAdapter>(
      adapter_result);
    const StopAdapterController controller{
      StopAdapterControllerConfig{true},
      adapter,
    };

    const auto result = controller.request_stop(valid_stop());

    EXPECT_EQ(adapter->invocation_count(), 1U);
    EXPECT_FALSE(result.adapter_accepted);
    EXPECT_EQ(result.state, StopAdapterState::Rejected);
    EXPECT_EQ(
      result.reason,
      "native_stop_adapter_contract_violation");
    expect_no_physical_effect(result);
  }
}

TEST(StopAdapterControllerTest, ContradictoryInvocationFailsClosed)
{
  auto adapter_result = accepted_result();
  adapter_result.adapter_invoked = false;
  auto adapter =
    std::make_shared<ConfigurableAdapter>(
    adapter_result);
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };

  const auto result = controller.request_stop(valid_stop());

  EXPECT_EQ(adapter->invocation_count(), 1U);
  EXPECT_FALSE(result.adapter_accepted);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_contract_violation");
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, ContradictoryRejectionReasonFailsClosed)
{
  auto adapter_result = accepted_result();
  adapter_result.adapter_accepted = false;
  adapter_result.state = StopAdapterState::Rejected;
  auto adapter =
    std::make_shared<ConfigurableAdapter>(
    adapter_result);
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };

  const auto result = controller.request_stop(valid_stop());

  EXPECT_EQ(adapter->invocation_count(), 1U);
  EXPECT_FALSE(result.adapter_accepted);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_contract_violation");
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, UnexpectedAcceptedResultFailsClosed)
{
  auto adapter_result = accepted_result();
  adapter_result.state = StopAdapterState::Rejected;
  adapter_result.reason = "unsafe arbitrary success reason";
  auto adapter =
    std::make_shared<ConfigurableAdapter>(
    adapter_result);
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };

  const auto result = controller.request_stop(valid_stop());

  EXPECT_FALSE(result.adapter_accepted);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_contract_violation");
  EXPECT_EQ(
    result.reason.find("unsafe"),
    std::string::npos);
  expect_no_physical_effect(result);
}

TEST(StopAdapterControllerTest, MismatchedResultIdentityFailsClosed)
{
  auto adapter_result = accepted_result();
  adapter_result.command_id = "different-command";
  auto adapter =
    std::make_shared<ConfigurableAdapter>(
    adapter_result);
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };

  const auto result = controller.request_stop(valid_stop());

  EXPECT_FALSE(result.adapter_accepted);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_contract_violation");
}

TEST(StopAdapterControllerTest, UnsafeAdapterNameFailsClosed)
{
  auto adapter_result = accepted_result();
  adapter_result.adapter_name = "unsafe adapter name";
  auto adapter =
    std::make_shared<ConfigurableAdapter>(
    adapter_result);
  const StopAdapterController controller{
    StopAdapterControllerConfig{true},
    adapter,
  };

  const auto result = controller.request_stop(valid_stop());

  EXPECT_FALSE(result.adapter_accepted);
  EXPECT_EQ(
    result.reason,
    "native_stop_adapter_contract_violation");
}

TEST(StopAdapterControllerTest, ProductionSourceHasNoWorkersOrTransports)
{
  const std::string source_path =
    std::string(SAVO_BRIDGE_SOURCE_DIR) +
    "/src/stop_command_adapter.cpp";
  std::ifstream input(source_path);
  ASSERT_TRUE(input.good());
  const std::string source{
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>()};

  for (const std::string forbidden : {
      "std::thread",
      "detach(",
      "retry",
      "rclcpp",
      "socket(",
      "AF_UNIX",
      "publish(",
    })
  {
    EXPECT_EQ(source.find(forbidden), std::string::npos)
      << forbidden;
  }
}

}  // namespace
