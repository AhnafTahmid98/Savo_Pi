# savo_base

Core-side Robot SAVO drivetrain package. The production C++ `base_driver_node`
converts the approved safe velocity command into mecanum wheel outputs through
the configured motor board. Python components remain for watchdog, heartbeat,
state publication, diagnostics, and approved fallback/development workflows.

## Safety boundary

- Runtime motion input is `/cmd_vel_safe`.
- `/safety/stop` and `/safety/slowdown_factor` are enforced by the configured
  base/control path.
- Base watchdog state and stop requests are published under `/savo_base/*`.
- Direct diagnostic access to PCA9685 motor outputs is not an approved test
  path.

Production bringup must start in `STOP`. Wheels-raised testing must follow
`../../../../docs/testing/base_test_plan.md` from the repository root.

## Build and test

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_base --symlink-install
source install/setup.bash
colcon test --packages-select savo_base --ctest-args --output-on-failure
colcon test-result --verbose
```
