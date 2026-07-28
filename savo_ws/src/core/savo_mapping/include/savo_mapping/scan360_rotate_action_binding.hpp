#ifndef SAVO_MAPPING__SCAN360_ROTATE_ACTION_BINDING_HPP_
#define SAVO_MAPPING__SCAN360_ROTATE_ACTION_BINDING_HPP_

#include "savo_mapping/scan360_orchestrator.hpp"
#include "savo_mapping/scan360_rotate_action_client.hpp"

namespace savo_mapping::scan360
{

[[nodiscard]] RotationClientSnapshot
map_scan360_rotate_action_snapshot(
  const ::savo_mapping::Scan360RotateActionClient::Snapshot &
  snapshot);

[[nodiscard]] Scan360RotationCallbacks
make_scan360_rotate_action_callbacks(
  ::savo_mapping::Scan360RotateActionClient::SharedPtr client);

}  // namespace savo_mapping::scan360

#endif  // SAVO_MAPPING__SCAN360_ROTATE_ACTION_BINDING_HPP_
