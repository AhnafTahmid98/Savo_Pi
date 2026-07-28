#include "savo_mapping/scan360_rotate_action_binding.hpp"

#include <stdexcept>
#include <utility>

namespace savo_mapping::scan360
{

RotationClientSnapshot
map_scan360_rotate_action_snapshot(
  const ::savo_mapping::Scan360RotateActionClient::Snapshot &
  snapshot)
{
  RotationClientSnapshot mapped;
  mapped.reason = snapshot.reason;

  using NativeState =
    ::savo_mapping::Scan360RotateActionClient::State;

  switch (snapshot.state) {
    case NativeState::kIdle:
      mapped.state = RotationClientState::Idle;
      break;

    case NativeState::kWaitingForServer:
    case NativeState::kWaitingForGoalResponse:
      mapped.state = RotationClientState::Pending;
      break;

    case NativeState::kActive:
      mapped.state = RotationClientState::Active;
      break;

    case NativeState::kCanceling:
      mapped.state = RotationClientState::Canceling;
      break;

    case NativeState::kSucceeded:
      mapped.state = RotationClientState::Succeeded;
      break;

    case NativeState::kCanceled:
      mapped.state = RotationClientState::Canceled;
      break;

    case NativeState::kRejected:
      mapped.state = RotationClientState::Rejected;
      break;

    case NativeState::kAborted:
    case NativeState::kTimedOut:
    case NativeState::kFailed:
      mapped.state = RotationClientState::Failed;
      break;
  }

  return mapped;
}

Scan360RotationCallbacks
make_scan360_rotate_action_callbacks(
  ::savo_mapping::Scan360RotateActionClient::SharedPtr client)
{
  if (!client) {
    throw std::invalid_argument(
            "scan360_rotate_action_client_missing");
  }

  Scan360RotationCallbacks callbacks;

  callbacks.request_rotation =
    [client](
    const double target_yaw_rad,
    const double max_duration_sec)
    {
      return client->request_rotation(
        target_yaw_rad,
        max_duration_sec);
    };

  callbacks.request_cancel =
    [client]()
    {
      return client->request_cancel();
    };

  callbacks.tick =
    [client]()
    {
      client->tick();
    };

  callbacks.snapshot =
    [client]()
    {
      return map_scan360_rotate_action_snapshot(
        client->snapshot());
    };

  return callbacks;
}

}  // namespace savo_mapping::scan360
