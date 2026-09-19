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

Producer hardware reads occur on the `publish_rate_hz` timer (1 Hz in the
production profiles). There is intentionally no independent
`sample_rate_hz`. Thresholds use the canonical `ups_*` and `base_*` names from
`config/power_common.yaml`; role-specific YAML must not contain aliases that
look configurable but are ignored.

Core and edge launches are separate:

```text
launch/power_core.launch.py
launch/power_edge.launch.py
```

`power_core.launch.py` always creates the required Core UPS, base battery,
aggregate, health, and dashboard nodes. Optional Edge health/dashboard nodes
are controlled by explicit launch arguments. The old profile-level
`enable_*` keys were not launch controls and are not part of the contract.

Do not assume percentage values are valid unless the corresponding source and
calibration state report them as valid. Power faults must remain visible to
bringup, supervisor, UI, and observer components.

## Core aggregate source policy

Core UPS and base-battery telemetry are always required by the Core power
aggregate. Edge UPS telemetry remains subscribed and visible, but is optional
for aggregate readiness by default. This supports installations where the Edge
UPS powers the Edge Pi correctly while its I2C telemetry path is unavailable or
unreliable; it does not fabricate or suppress Edge readings.

Start with the default policy:

```bash
ros2 launch savo_power power_core.launch.py
```

Explicitly require Edge UPS telemetry for strict operation:

```bash
ros2 launch savo_power power_core.launch.py edge_ups_expected:=true
```

The generic role-selecting bringup exposes the same option:

```bash
ros2 launch savo_power power_bringup.launch.py \
  role:=core \
  edge_ups_expected:=false

ros2 launch savo_power power_bringup.launch.py \
  role:=core \
  edge_ups_expected:=true
```

## Build and test

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_power --symlink-install
source install/setup.bash
colcon test --packages-select savo_power --ctest-args --output-on-failure
colcon test-result --verbose
```
