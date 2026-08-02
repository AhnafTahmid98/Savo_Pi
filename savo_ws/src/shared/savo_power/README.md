# savo_power

Shared Robot SAVO power-monitoring package. The production C++ nodes monitor:

- core UPS status;
- edge UPS status;
- Freenove/base battery status;
- aggregate power health and shutdown requests.

Canonical outputs include `/savo_power/core/ups`, `/savo_power/edge/ups`,
`/savo_power/base/battery`, `/savo_power/status`, `/savo_power/health`, and the
read-only dashboard topics. Python implementations remain as configured
fallback and diagnostic paths.

Core and edge launches are separate:

```text
launch/power_core.launch.py
launch/power_edge.launch.py
```

Do not assume percentage values are valid unless the corresponding source and
calibration state report them as valid. Power faults must remain visible to
bringup, supervisor, UI, and observer components.

## Build and test

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_power --symlink-install
source install/setup.bash
colcon test --packages-select savo_power --ctest-args --output-on-failure
colcon test-result --verbose
```
