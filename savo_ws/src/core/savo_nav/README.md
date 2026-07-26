# savo_nav

Robot Savo core-side navigation package for ROS 2 Jazzy and Nav2.

## Current development phase

Phase 0B — Frozen C++ and YAML contracts.

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
