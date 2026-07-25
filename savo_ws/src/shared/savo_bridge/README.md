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
