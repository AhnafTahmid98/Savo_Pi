# savo_bridge

`savo_bridge` is the native C++ boundary between the Robot Savo ROS 2
system and SavoMind.

## Runtime ownership

The package source is stored under `src/shared`, but the production
bridge process will run once on `savo-edge`.

ROS 2 DDS over the direct Ethernet connection already connects
`savo-core` and `savo-edge`. `savo_bridge` is not a separate network
bridge between the two Raspberry Pis.

## Responsibilities

The completed package will own:

- verified ROS topic subscriptions
- subscriber QoS
- ROS graph and DDS visibility evidence
- message validation
- monotonic freshness tracking
- bridge health
- stable ROS-independent runtime snapshots for SavoMind
- later command validation and ROS clients in separately approved phases

Robot subsystem packages remain authoritative for their own state and
behaviour.

## Initial safety boundary

The initial bridge is strictly read-only.

These paths remain disabled:

- movement dispatch
- teleoperation dispatch
- navigation dispatch
- mapping commands
- direct motor commands
- behaviour-changing service calls
- ROS action-goal dispatch
- SavoMind command processing

## Phase 1A-1

Phase 1A-1 establishes only:

- the native `ament_cmake` package
- a reusable C++ core library
- the package version contract
- strict compiler warnings
- C++ unit testing
- lint integration
- installation and export rules

It does not contain a ROS node, publisher, subscription, service,
action client, runtime snapshot writer or SavoMind transport.

## Production target

- Ubuntu 24.04 ARM64
- Raspberry Pi 5
- ROS 2 Jazzy
- modern C++
- one native bridge process on `savo-edge`

## Phase 1C graph discovery

`savo_bridge` performs read-only ROS graph discovery. It does not
subscribe to subsystem data and does not create service or action
clients.

Local DDS activity requires the bridge node and all bridge-owned
status topics to appear in the graph. Core and edge visibility use
explicit node and topic selector parameters:

- `core_evidence_nodes`
- `core_evidence_topics`
- `edge_evidence_nodes`
- `edge_evidence_topics`

Selectors are empty by default. Graph presence is evidence of a
configured public ROS entity, not proof of the physical hostname
running that entity. Commands remain disabled and bridge readiness
remains false during this phase.

## Phase 2A runtime snapshot publication

The native bridge can atomically publish a canonical read-only runtime
snapshot for SavoMind.

Parameters:

- `snapshot_enabled` defaults to `false`.
- `snapshot_path` defaults to `/run/savo_bridge/snapshot.json`.

The parent directory must already exist and be writable by the native
bridge process. The bridge does not create or change runtime-directory
ownership.

When enabled, the bridge writes once per status cycle using the tested
same-directory temporary-file, `fsync`, atomic rename and parent
directory `fsync` contract.

During Phase 2A the snapshot contains no subsystem observations, so its
derived health remains `unknown` with reason `no_topics`.
`bridge_ready` remains false and commands remain disabled.

