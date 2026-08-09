# savo_bridge

## Purpose

Typed, bounded Edge-local ROS 2 ↔ SavoMind boundary; not a generic ROS proxy.

## Deployment

Edge production package. `edge_bridge.launch.py` starts `savo_bridge_node`; runtime sockets live under `/run/savo_bridge`.

## Responsibilities

Write a bounded telemetry snapshot and accept explicitly modeled STOP, teleop, navigation/cancel, mapping control/query, and supervisor observation commands over a Unix socket.

## Non-responsibilities and authority boundaries

No shell execution, arbitrary topic/service/action forwarding, direct motor access, navigation/readiness bypass, map/location approval, or generic ROS graph proxy. SavoMind has no ROS authority beyond typed adapters.

## Package structure

C++ protocol, command server/dispatcher, graph evidence, freshness/health, observation, snapshot, and STOP adapter; scripts/systemd assets and contract tests.

## Runtime components

### `savo_bridge_node`

Production C++ node. Discovers configured evidence, subscribes observations, writes snapshots, binds `/run/savo_bridge/command.sock`, checks peer credentials, deduplicates command IDs, and dispatches bounded ROS operations.

## Runtime data flow

`ROS observations -> snapshot.json -> SavoMind`; `typed socket request -> freshness/authority/map checks -> ROS adapter`.

## ROS interfaces

### Published topics

Only explicit command adapters: `/savo_control/mode_cmd`, `/savo_control/external_stop`, and bounded `/cmd_vel_manual`; never `/cmd_vel_safe`. Other operations use typed services/actions.

### Subscribed topics

Configured observations include control mode, safety stop, `/cmd_vel_safe`, nav readiness/map context, locations status, speech readiness, mapping status, and supervisor summary.

### Services

Clients include `/savo_locations/resolve`, `/savo_mapping/autonomous/control`, and configured query dependencies.

### Actions

Clients use guarded navigation `/savo_nav/navigation/navigate_to_pose` and mapping `/savo_mapping/autonomous/run`; cancellation remains typed/bounded.

## TF ownership

None.

## Parameters and configuration

Socket default `/run/savo_bridge/command.sock`, mode `0660` (`432` decimal), group/allowed UID `10001`, request/response max 65536 bytes, observed-state timeout 1000 ms, STOP confirmation 2000 ms, active map context required.

## Launch files

`edge_bridge.launch.py` loads `savo_bridge.edge.yaml`.

## Persistent state and runtime files

Ephemeral socket and `/run/savo_bridge/snapshot.json`; no long-term database.

## Hardware ownership

None.

## Dependencies

### Internal Robot Savo dependencies

Control, perception, nav, locations, mapping, supervisor, speech, and shared messages.

### External ROS/system dependencies

Unix-domain sockets/peer credentials, JSON/protocol support, ROS client/action APIs.

## Safety behavior

Malformed/oversized/duplicate/stale/unauthorized requests, wrong peer UID, absent services/actions, missing readiness/map context, or timeout fail closed. STOP has explicit confirmation behavior.

## Failure and degraded behavior

Bridge loss removes SavoMind commands but cannot remove Core STOP/safety. Snapshot may become stale and is labeled/rejected accordingly.

## Startup and shutdown behavior

Creates socket with configured ownership after runtime preparation; unlinks it on clean shutdown. Edge bringup enables bridge by default.

## Build

`bash deploy/edge/build_edge.sh --clean --test`

## Run

`ros2 launch savo_bridge edge_bridge.launch.py`

## Validation and testing

Protocol, credentials, limits, freshness, graph evidence, snapshot, STOP, navigation/cancel, teleop, mapping, and fail-closed adapter tests.

## Current validation status

Implemented/source-validated with dated SavoMind protocol smoke evidence; live Edge socket ownership/latency/recovery integration required.

## Known limitations and remaining validation

Configured UID/GID must match deployed SavoMind account; runtime map context and DDS reachability must be verified.

## Change-control considerations

Every new command requires explicit schema, bounds, freshness/authority tests, and boundary review; generic forwarding is prohibited.

## Related documentation

- [Implementation README](../../savo_ws/src/shared/savo_bridge/README.md)
- [Speech/intent flow](../architecture/speech_intent_flow.md)
- [Edge architecture](../architecture/savo_edge_architecture.md)
- [Ownership matrix](package_ownership_matrix.md)

