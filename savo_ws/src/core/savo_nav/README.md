# savo_nav

`savo_nav` is Robot SAVO's core-side ROS 2 Jazzy navigation package. It owns
Nav2 orchestration, guarded goal admission, saved-map and live-mapping
navigation, production-map activation, navigation readiness, bounded recovery
coordination, named-location navigation, and coverage-path forwarding.

Production runtime code is C++17. Python is used only for ROS launch, tests, and
the isolated location-integration smoke tool. The deployment target is the
Raspberry Pi 5 ARM64 `savo-core` computer.

## Safety and ownership boundary

Nav2 may publish velocity only through:

```text
Nav2 -> /cmd_vel_nav -> savo_control -> /cmd_vel
     -> savo_perception safety gate -> /cmd_vel_safe -> savo_base
```

`savo_nav` never publishes directly to `/cmd_vel`, `/cmd_vel_safe`, or
`/cmd_vel_recovery`.

The public action servers are protected by the control/recovery guard and the
goal-admission gate. Internal gateway actions are hidden under
`/savo_nav/_internal/...`; callers use only the public guarded interfaces.

Motion-producing recovery remains owned by `savo_control`. The package-owned
Nav2 behavior trees may replan, clear costmaps, and wait, but do not invoke
Nav2 `Spin`, `BackUp`, `DriveOnHeading`, or assisted-teleoperation behaviors.

## TF authority

Saved-map navigation:

```text
map --AMCL--> odom --savo_localization--> base_footprint
    --robot_state_publisher--> base_link
```

Live-mapping navigation:

```text
map --SLAM Toolbox--> odom --savo_localization--> base_footprint
    --robot_state_publisher--> base_link
```

AMCL and SLAM Toolbox must never publish `map -> odom` simultaneously.

## Implemented production runtime

The package provides:

- `navigation_readiness_node`;
- `active_map_context_node`;
- `control_recovery_guard_node`;
- `goal_admission_gate_node`;
- `goal_gateway_node`;
- `navigate_to_location_node`;
- guarded mission/location and exploration `NavigateToPose` forwarding;
- guarded coverage-path validation and Nav2 `FollowPath` forwarding;
- goal arbitration, cancel acknowledgement, watchdogs, and stale-result handling;
- saved-map Nav2 bringup with AMCL and map server;
- live-mapping Nav2 bringup without AMCL or map server;
- fail-closed startup from the active AM-8 joint release;
- immutable release-artifact size and SHA-256 verification at every production
  startup;
- synchronization of the active map ID, revision, and release ID with
  `savo_supervisor`;
- readiness blocking while production map-context synchronization is missing,
  mismatched, stale, or unavailable;
- separate package-owned normal-navigation and exploration behavior trees;
- a LiDAR-only controlled-test costmap profile; and
- a source-complete filtered-D435 local VoxelLayer companion profile.

## Behavior trees

Normal saved-map and named-location goals use:

```text
behavior_trees/navigate_to_pose.xml
```

Frontier-exploration goals use:

```text
behavior_trees/exploration_navigation.xml
```

External callers cannot provide arbitrary behavior-tree paths. The gateway
selects the correct installed tree from the trusted goal source.

## Launch files

### Production navigation from AM-8

Use this for a released production map:

```bash
ros2 launch savo_nav production_navigation.launch.py
```

The default production root is:

```text
/var/lib/robot_savo/maps/production
```

Before Nav2 is launched, the production entry point verifies:

1. `active_map.yaml` schema and active state;
2. release directory confinement under the production root;
3. joint release manifest schema v2 and immutable status;
4. active map ID, frame, release ID, and positive map revision;
5. quality pass and explicit operator approval;
6. every required release artifact's path, size, and SHA-256;
7. map YAML and map-image consistency;
8. location-snapshot digest;
9. the installed geometry profile is physically locked; and
10. the installed and released geometry digests match.

It then starts saved-map Nav2 with the verified map identity and synchronizes
that identity with `savo_supervisor`. Production readiness remains blocked
until the synchronization node is alive and reports the exact same map ID,
revision, and release ID.

Custom storage can be supplied explicitly:

```bash
ros2 launch savo_nav production_navigation.launch.py \
  production_map_root:=/custom/maps/production
```

### Controlled saved-map navigation

Use this for development maps that are not yet an active AM-8 release:

```bash
ros2 launch savo_nav saved_map_navigation.launch.py \
  map:=/absolute/path/to/map.yaml \
  map_id:=campus_main \
  map_revision:=1 \
  autostart:=false
```

This generic launch defaults to lifecycle autostart disabled and does not
require supervisor map-context synchronization. It is not a substitute for the
production release entry point.

### Live-mapping navigation

```bash
ros2 launch savo_nav live_mapping_navigation.launch.py \
  autostart:=true
```

Use this only while SLAM Toolbox already publishes `/map` and `map -> odom`.
The launch intentionally excludes AMCL and map server. This is the Nav2 path
used by frontier exploration during an active mapping mission.

Before motion, run the read-only graph preflight:

```bash
ros2 run savo_nav run_mapping_nav_preflight --mode live
```

After deliberately changing `savo_control` to `NAV`, require the complete
readiness chain before sending a goal:

```bash
ros2 run savo_nav run_mapping_nav_preflight --mode live --expect-ready
```

## Costmap and readiness profiles

### Guarded LiDAR baseline

Use:

```text
config/nav2_saved_map.yaml
config/readiness.yaml
```

This profile requires the map, complete TF chain, fused odometry, LiDAR, Nav2
action server, global and local costmaps, `NAV` control mode, and fresh safety
state. It is the controlled first-test profile.

Both costmaps retain the Nav-owned conservative `0.330 x 0.240 m` collision
envelope with `0.02 m` padding. The generated `savo_description` footprint is
the measured plate envelope only and is not the production collision footprint.
LiDAR marks and clears obstacles in the global and local costmaps.

### Filtered D435 VoxelLayer companion

Use only after real D435 validation:

```text
config/nav2_saved_map_voxel.yaml
config/readiness_realsense_voxel.yaml
```

Example production selection after validation:

```bash
ros2 launch savo_nav production_navigation.launch.py \
  params_file:=$(ros2 pkg prefix savo_nav)/share/savo_nav/config/nav2_saved_map_voxel.yaml \
  readiness_params:=$(ros2 pkg prefix savo_nav)/share/savo_nav/config/readiness_realsense_voxel.yaml
```

The companion profile consumes only:

```text
/savo_perception/obstacles/points
```

The raw RealSense cloud is forbidden. The filtered obstacle-only cloud marks
local obstacles without clearing. LiDAR remains the authoritative clearing
source, and no D435 layer is added to the global costmap.

## Build and test

Development build:

```bash
cd ~/Savo_Pi/savo_ws
set +u
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-up-to savo_nav savo_mapping savo_supervisor \
  --symlink-install \
  --event-handlers console_direct+

source install/setup.bash

colcon test \
  --packages-select savo_nav \
  --event-handlers console_direct+

colcon test-result --verbose
```

Production Pi build after tests pass:

```bash
colcon build \
  --packages-up-to savo_nav savo_mapping savo_supervisor \
  --cmake-args -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
```

Disabling `BUILD_TESTING` prevents test fixtures from entering the Pi install.

## Controlled real-robot test order

1. Measure, review, and lock the real geometry profile.
2. Build and run all registered tests.
3. Start description, base, localization, LiDAR, perception safety,
   `savo_control`, and `savo_supervisor`.
4. Confirm the TF chain and `/odometry/filtered` are healthy.
5. Perform wheels-raised command-chain and stop tests.
6. Use the LiDAR-only profile for short, low-speed saved-map goals.
7. Validate lateral mecanum tracking, cancellation, goal tolerances, obstacle
   stopping, stale dependencies, and recovery handoff.
8. Run autonomous mapping and complete a real AM-8 joint release.
9. Start `production_navigation.launch.py` and verify the same release survives
   process restart and Pi reboot.
10. Validate the filtered D435 producer separately before selecting the voxel
    companion profile.

## Completion boundary

The `savo_nav` source, contracts, production release integration, behavior-tree
ownership, and optional D435 configuration are complete. Remaining work is
physical validation and tuning, not missing package architecture:

- lock the measured robot geometry;
- tune DWB limits and critics on the loaded robot;
- tune AMCL and goal tolerances on real maps;
- validate obstacle, cancellation, and recovery behavior on the floor;
- calibrate the filtered D435 self-filter and voxel limits; and
- complete the real-robot production-release restart/recovery test.
