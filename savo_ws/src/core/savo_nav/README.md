# savo_nav

Robot Savo core-side navigation package for ROS 2 Jazzy and Nav2.

## Current development phase

Phase 6 — Goal gateway and Nav2 action forwarding.

The obsolete Python navigation scaffold has been removed. This phase establishes
the package identity, ownership boundaries, directory layout, build conventions,
and cross-package contracts.

No runtime navigation executable is provided in Phase 0.

## Package conventions

- Package version: `0.1.0`
- ROS distribution: ROS 2 Jazzy
- Package format: 3
- Build system: `ament_cmake`
- Production language: C++17
- Compiler warnings: `-Wall -Wextra -Wpedantic`
- License: Proprietary
- Runtime target: Raspberry Pi 5 ARM64 on `savo-core`

## Package ownership

`savo_nav` owns:

- Nav2 orchestration
- Navigation lifecycle integration
- Saved-map navigation integration
- AMCL configuration and initialization policy
- Global path planning
- Local path following
- Global and local Nav2 costmaps
- RPLIDAR obstacle integration
- RealSense three-dimensional VoxelLayer integration
- Validated navigation-goal execution
- Navigation readiness and state
- Bounded recovery coordination
- Navigation observer topics and markers

`savo_nav` does not own:

- Wheel odometry
- IMU processing
- Visual odometry
- EKF sensor fusion
- Motor kinematics
- Motor hardware
- Command-source arbitration
- Final velocity safety gating
- ToF or ultrasonic safety policy
- Map creation
- Frontier selection
- Named-location storage
- AprilTag detection
- Robot-wide mission supervision

## Locked navigation command chain

Nav2 publishes navigation velocity only to:

    /cmd_vel_nav

The complete command path is:

    Nav2
      -> /cmd_vel_nav
      -> savo_control
      -> /cmd_vel
      -> savo_perception safety gate
      -> /cmd_vel_safe
      -> savo_base
      -> motor hardware

Direct publication from `savo_nav` to any of the following is forbidden:

    /cmd_vel
    /cmd_vel_safe
    /cmd_vel_recovery

## Locked TF target

The production TF chain is:

    map
      -> odom
        -> base_footprint
          -> base_link

Ownership:

- `map -> odom`: AMCL during saved-map navigation
- `map -> odom`: SLAM Toolbox during live mapping
- `odom -> base_footprint`: `savo_localization`
- `base_footprint -> base_link`: `robot_state_publisher`

Only one component may publish `map -> odom` at a time.

The current localization package must be corrected from `base_link` to
`base_footprint` before production Nav2 runtime integration.

## RealSense three-dimensional obstacle path

The planned production data flow is:

    Intel RealSense D435
      -> savo_realsense raw PointCloud2
      -> savo_perception obstacle-cloud filtering
      -> /savo_perception/obstacles/points
      -> savo_nav local Nav2 VoxelLayer

Raw RealSense point-cloud topic:

    /camera/camera/depth/color/points

Production filtered obstacle-cloud topic:

    /savo_perception/obstacles/points

## Mapping integration

`savo_mapping` chooses exploration goals.

It sends those goals through:

    /savo_nav/exploration/navigate_to_pose

`savo_mapping` does not publish navigation velocity and does not bypass the
`savo_nav` goal gateway.

## Mac RViz observer

RViz runs only on the MacBook Air M3 Ubuntu 24.04 ARM64 virtual machine.

The Mac observes:

- Robot movement
- TF
- Wheel odometry
- Visual odometry
- Fused odometry
- Localization drift
- AMCL pose
- Navigation goals
- Global and local paths
- Global and local costmaps
- LiDAR
- RealSense point clouds
- Voxel obstacles
- Mapping frontiers
- AprilTag detections
- Navigation readiness and state

The Mac must not publish:

- Navigation goals
- Initial poses
- Teleoperation commands
- Velocity commands
- Control-mode commands
- Lifecycle commands
- Safety commands
- Robot TF

Neither Raspberry Pi runs RViz.

## Documentation

Detailed architecture and interface documentation remains in the separate
workspace-level documentation folder.


## Phase 0B contract files

The following files freeze contract version 1:

- `include/savo_nav/constants.hpp`
- `include/savo_nav/topic_names.hpp`
- `include/savo_nav/action_names.hpp`
- `include/savo_nav/service_names.hpp`
- `include/savo_nav/frame_names.hpp`
- `config/topics.yaml`
- `config/action_servers.yaml`
- `config/frames.yaml`

The C++ constants and YAML contracts are checked for consistency by
`test/contracts/test_phase0_contracts.py`.

## Phase 1 implementation

Phase 1 adds:

- `savo_nav_core`, the first hardware-independent C++ library
- `navigation_readiness_node`, the first ROS 2 executable
- stable navigation readiness states and result types
- readiness and reason publication
- heartbeat publication
- C++ unit tests for readiness-state invariants

The node remains in the `starting` state during Phase 1. Real dependency
monitoring is implemented in the later navigation-readiness phase.

## Phase 2 implementation

Phase 2 adds hardware-independent contracts for:

- common validation results and stable validation codes
- navigation goal identity, source and map-frame invariants
- saved-map and live-mapping context ownership
- AMCL versus SLAM Toolbox `map -> odom` exclusivity
- navigation state classification and allowed transitions
- terminal and pending navigation-result invariants
- cancel acknowledgement outcomes, including late success

These contracts do not start Nav2, accept goals or publish velocity commands.

## Phase 3 implementation

Phase 3 adds a hardware-independent navigation-readiness core.

Hardware and sensor ownership remains outside `savo_nav`. The readiness node
subscribes to published ROS interfaces from mapping, localization, LiDAR,
perception and control. It also observes the internal Nav2 action server and
costmaps.

The readiness evaluator checks:

- safety stop and slowdown freshness
- navigation control-mode permission
- map availability
- `map -> odom -> base_footprint -> base_link`
- filtered localization odometry
- LiDAR scan freshness
- filtered RealSense obstacle-point-cloud freshness
- Nav2 NavigateToPose action availability
- global and local costmap freshness

It does not publish motor, velocity, lifecycle or hardware commands.

## Phase 4 implementation

Phase 4 adds deterministic, hardware-independent goal validation and
single-active-goal arbitration.

Validation covers:

- stable goal identity and source
- sequence validity
- map-frame enforcement
- finite and bounded target coordinates
- normalized target yaw
- active-map availability
- saved-map identity matching
- navigation readiness permission
- rejection of cancellation flags on new goals

Arbitration covers:

- exactly one active goal
- no silent replacement or priority-based preemption
- deterministic source priorities
- duplicate goal-ID protection
- per-source stale-sequence protection
- bounded terminal-goal history
- cancel-request ownership
- cancel acknowledgement before release
- late completion after a cancellation request

Phase 4 does not send goals to Nav2 and does not publish velocity or hardware
commands.

## Phase 5 implementation

Phase 5 adds the baseline saved-map Nav2 launch and parameter stack:

- Map Server
- AMCL with the omnidirectional motion model
- Navfn global planner
- holonomic DWB local controller
- global and local LiDAR-only 2D costmaps
- Behavior Server
- Behavior Tree Navigator
- Waypoint Follower
- separate localization and navigation lifecycle managers
- `/cmd_vel_nav` as the only Nav2 velocity output
- lifecycle autostart disabled by default
- isolated configure-only Nav2 dry-run validation

This phase does not add the public Savo goal gateway, automatic supervisor
activation, RealSense VoxelLayer, hardware movement or RViz ownership.

The configured `robot_radius` is provisional and must be replaced with a
measured footprint or verified radius before real-hardware navigation.

## Phase 6 implementation

Phase 6 adds two public `NavigateToPose` action servers:

- `/savo_nav/navigation/navigate_to_pose`
- `/savo_nav/exploration/navigate_to_pose`

Both are owned by one deterministic gateway and forward to Nav2's internal
`/navigate_to_pose` action only after readiness, map-context, pose and
single-active-goal checks pass.

The gateway forwards Nav2 feedback and terminal results to the original
client. External cancellation is forwarded to Nav2, but ownership is retained
until Nav2 returns a terminal result. Late success after a cancellation request
is supported.

Arbitrary behavior-tree overrides are disabled by default. Execution-time and
stale-feedback watchdogs request Nav2 cancellation.

The gateway publishes observer-only state, status, feedback and result topics.
It does not publish velocity, lifecycle or hardware commands.
