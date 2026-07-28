// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SUPPORT__RECORDING_STOP_ADAPTER_HPP_
#define SUPPORT__RECORDING_STOP_ADAPTER_HPP_

#include <cstddef>
#include <optional>

#include "savo_bridge/stop_command_adapter.hpp"

namespace savo_bridge::test
{

class RecordingStopAdapter final : public StopCommandAdapter
{
public:
  [[nodiscard]] std::size_t invocation_count() const noexcept
  {
    return invocation_count_;
  }

  [[nodiscard]] const std::optional<StopAdapterRequest> &
  last_request() const noexcept
  {
    return last_request_;
  }

protected:
  [[nodiscard]] StopAdapterResult request_stop_impl(
    const StopAdapterRequest & request) override
  {
    ++invocation_count_;
    last_request_ = request;

    StopAdapterResult result;
    result.adapter_invocation_attempted = true;
    result.adapter_invoked = true;
    result.adapter_accepted = true;
    result.state = StopAdapterState::TestNoEffect;
    result.reason = "native_stop_test_adapter_accepted";
    result.adapter_name = "recording_stop_adapter";
    result.command_id = request.command_id;
    return result;
  }

private:
  std::size_t invocation_count_{0U};
  std::optional<StopAdapterRequest> last_request_;
};

}  // namespace savo_bridge::test

#endif  // SUPPORT__RECORDING_STOP_ADAPTER_HPP_
