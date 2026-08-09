# savo_ui

## Purpose

Read-only Edge-local framebuffer/touch presentation of robot, voice, safety, navigation, mapping, locations, power, and camera-preview state.

## Deployment

Edge only and optional (`start_ui=false`). Current workspace contains classic production `ui_node` plus a non-destructive selectable `ui_v2_node`; selector defaults to classic.

## Responsibilities

Render an 800×480 interface, consume live state with freshness labels, show safety/power/navigation/mapping/location/speech state, route local display pages, and support dry-run preview frames.

## Non-responsibilities and authority boundaries

No motion, mission, map/location approval, navigation goal, ROS service/action client, or shell authority. Touch changes presentation only.

## Package structure

C++ classic UI and current v2 source, Python legacy display/camera/mode/debug helpers, assets, role profiles, launch/systemd, and contract tests.

## Runtime components

### `ui_node`

Classic production C++ framebuffer/touch UI with full system subscriptions and stale-state handling.

### `ui_v2_node`

Alternative C++ voice/navigation display. `ui_select.launch.py` chooses `classic|v2`; v2 profiles select PC dry-run or Edge hardware. It does not replace classic.

### Legacy helpers

`display_manager_node`, `ui_mode_router_node`, `ui_debug_node`, `real_cam_node` and view/fake-camera scripts support older composition/testing.

## Runtime data flow

`read-only ROS telemetry -> freshness model/page arbitration -> framebuffer or preview PNG`; touch -> local page state only.

## ROS interfaces

### Published topics

Classic/v2 production renderers publish no robot command. Legacy helper status/preview topics are local presentation contracts.

### Subscribed topics

Core/Edge bringup state; control mode; safety; nav state/status/feedback/result; typed `AutonomousMappingStatus`; locations status and `LocationEvent`; bridge state/readiness; power; speech readiness/state/dashboard/transcript/response/playback; configured camera preview state.

### Services

None.

### Actions

None.

## TF ownership

None.

## Parameters and configuration

Classic uses framebuffer `/dev/fb0`, configured touch device/assets, `800x480`, loop/freshness/power-history and camera-preview policy. V2 uses 30 Hz, `page_mode=auto|voice|navigation`, 4 s arrival hold, framebuffer disabled/preview enabled in base config with Edge profile overrides.

## Launch files

`ui_bringup.launch.py` classic; `ui_v2_bringup.launch.py` alternative; `ui_select.launch.py` defaults classic; `savo_ui_bringup.launch.py` is legacy multi-node composition.

## Persistent state and runtime files

No robot state. Dry-run/v2 may export previews under `/tmp`; production assets are installed read-only.

## Hardware ownership

Edge framebuffer/display and touch input. Camera preview observes a camera source but does not own camera hardware.

## Dependencies

### Internal Robot Savo dependencies

Read-only consumers of bringup/control/perception/nav/mapping/locations/bridge/power/speech and `savo_msgs`.

### External ROS/system dependencies

Linux framebuffer/input devices, image/rendering libraries, ROS messages.

## Safety behavior

Stale/malformed inputs are labeled unavailable/stale; safety overlays take presentation priority. UI cannot clear stops or send commands.

## Failure and degraded behavior

UI failure only removes local presentation. Missing topics degrade individual panels and cannot affect robot authority.

## Startup and shutdown behavior

Disabled by default; validates profile/devices, renders, and restores/releases display/input resources on shutdown.

## Build

`bash deploy/edge/build_edge.sh --clean --test`.

## Run

`ros2 launch savo_ui ui_select.launch.py ui_variant:=classic profile:=pi`.

## Validation and testing

Classic live-state/integration tests and current v2 source/config/launch/read-only contract test.

## Current validation status

Classic implemented/source-tested; current uncommitted v2 workspace path is source-contract tested but not production-selected. Physical 800×480 framebuffer, touch, stale overlays, and page behavior require Edge validation.

## Known limitations and remaining validation

Camera preview is not production authority and remains profile-dependent. V2 currently focuses voice/navigation, not full classic parity. The legacy Python `ui_mode_router_node.py` still observes `/savo_intent/intent_result`; there is no current producer or `savo_intent` package, and this compatibility subscription is not part of the C++ classic/v2 production authority path.

## Change-control considerations

Preserve read-only contract; selector/default, framebuffer/touch devices, freshness, and safety overlay changes require display regression.

## Related documentation

- [Implementation README](../../savo_ws/src/edge/savo_ui/README.md)
- [Edge architecture](../architecture/savo_edge_architecture.md)
- [Edge component validation](../testing/savo_edge_component_validation.md)
- [UI test plan](../testing/ui_test_plan.md)
- [Ownership matrix](package_ownership_matrix.md)
