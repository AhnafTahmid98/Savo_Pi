# savo_control

Core-side Robot SAVO control and motion-ownership package. It provides the C++
control-mode manager, velocity-source mux, command shaper, recovery manager,
backup escape, stuck detection, distance approach, and rotate-to-heading
components. Existing Python status and test helpers remain where used.

## Production rules

- Startup mode is `STOP`.
- Manual, navigation, recovery, and approved test commands enter through their
  dedicated inputs and are selected by the control-mode/mux contract.
- Output toward the safety/base path is `/cmd_vel_safe`.
- External STOP commands use `/savo_control/external_stop`; the mode manager
  publishes its authoritative current value on `/savo_control/external_stop_state`.
- Diagnostics must never bypass the control package or publish directly to
  motor hardware.

Use `launch/control_bringup.launch.py` for the canonical package bringup. Test
launches are not production startup entry points.

## Build and test

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_control --symlink-install
source install/setup.bash
colcon test --packages-select savo_control --ctest-args --output-on-failure
colcon test-result --verbose
```
