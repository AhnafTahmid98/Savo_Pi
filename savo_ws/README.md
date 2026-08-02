# Robot SAVO ROS 2 workspace

This is the ROS 2 Jazzy workspace for Robot SAVO. Production deployment uses
two Raspberry Pi 5 computers:

- `savo-core`: drivetrain, safety, LiDAR, localization, mapping, navigation,
  locations, supervisor, head, and core/base power;
- `savo-edge`: RealSense, visual odometry, speech hardware, touchscreen UI,
  bridge to SavoMind, and edge power.

Shared packages provide interfaces, robot description, perception contracts,
bringup, power, supervision, and read-only observation.

## Build

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Do not treat a successful build as permission for movement. Production startup
remains `STOP`; physical geometry, safety, sensors, and authorization must be
validated according to `../docs/testing/full_robot_test_plan.md`.

## Pre-real-test validation

```bash
cd ~/Savo_Pi
deploy/common/validate_full_bringup.sh
deploy/observer/validate_observer.sh
deploy/common/validate_pre_real_test_readiness.sh
```

The final validator may report `BLOCKED` for honest external or physical gates
such as provisional geometry, unavailable target dependencies, or the missing
production SavoMind speech endpoint.

## Final hardware-free regression

From the repository root on a ROS 2 Jazzy development computer:

```bash
deploy/common/run_pre_real_test_regression.sh --clean-affected
```

The script builds and tests the affected core, edge, shared, bringup, and
observer packages without launching hardware.
