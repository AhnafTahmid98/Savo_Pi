# Robot Savo full bringup closure — 2026-08-01

## Outcome

The shared `savo_bringup` package now owns complete distributed launch
orchestration while production runtime authority remains C++.

## Runtime ownership

### Core

- `savo_description`
- `savo_base`
- `savo_lidar`
- `savo_perception`
- `savo_localization`
- `savo_control`
- `savo_head`
- `savo_power`
- `savo_locations`
- `savo_mapping`
- `savo_nav`
- `savo_supervisor`

### Edge

- `savo_realsense`
- `savo_vo`
- optional filtered obstacle cloud
- `savo_speech`
- `savo_ui`
- `savo_bridge`
- edge `savo_power`

## Entry points

```bash
ros2 launch savo_bringup robot_bringup.launch.py host_role:=core
ros2 launch savo_bringup robot_bringup.launch.py host_role:=edge
```

Supported modes:

- `safe_idle`
- `manual`
- `manual_mapping`
- `autonomous_mapping`
- `saved_map_navigation`
- `diagnostics`

Supported profiles:

- `bench`
- `lidar_only`
- `lidar_d435_voxel`
- `production`

## Fail-closed rules

- motion-capable non-bench profiles require locked geometry;
- production never permits provisional geometry;
- the D435 voxel profile requires explicit hardware validation;
- SLAM and saved-map AMCL/Nav2 are selected by mutually exclusive modes;
- saved-map navigation enters through the verified AM-8 release launch;
- core control defaults to `STOP`;
- autonomous mapping starts only through `RunAutonomousMapping` contract v2;
- the semantic review gateway is enabled;
- map quality approval is required for the production AM-8 example.

## Readiness authority

The C++ `bringup_readiness_node` publishes independent core and edge state:

```text
/savo_bringup/core/state
/savo_bringup/core/ready
/savo_bringup/core/heartbeat
/savo_bringup/core/diagnostics

/savo_bringup/edge/state
/savo_bringup/edge/ready
/savo_bringup/edge/heartbeat
/savo_bringup/edge/diagnostics
```

It aggregates existing package authorities; it does not replace safety,
supervisor, AM-8, navigation readiness, or goal admission.

## First real test profile

Use `lidar_only`. Keep `start_obstacle_cloud:=false` and
`d435_voxel_validated:=false` until the filtered D435 cloud, self-filter, and
vertical collision envelope are validated.

## Local validation required

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_bringup --symlink-install
source install/setup.bash
colcon test --packages-select savo_bringup --event-handlers console_direct+
colcon test-result --verbose
```

Before running core bringup:

```bash
cd ~/Savo_Pi
sudo deploy/core/prepare_runtime_storage.sh \
  --owner "$USER" \
  --group "$USER"
```
