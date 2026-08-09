# UPS HAT and power setup

## Purpose and scope

This guide commissions non-actuating power telemetry on Core and Edge. Both UPS HATs use I2C bus 1 at address `0x36` (decimal 54). Core also reads the base-battery ADS7830 at `0x48` (decimal 72), channel 2, PCB profile `v2`.

Current source does not require the obsolete `/opt/x120x` service, an EEPROM overlay procedure, or an external fan daemon. Those older instructions are not part of Robot Savo's supported power path.

## Prerequisites and detection

- Power wiring and HAT orientation match the [power architecture](../hardware/power_architecture.md).
- Role build/test passes and no other I2C diagnostic owns the bus.
- The deployment user can open `/dev/i2c-1` under the target host policy.
- Automatic shutdown remains disabled.

```bash
id
ls -l /dev/i2c-1
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
```

The repository installer does not explicitly install `i2c-tools`. If `i2cdetect` is already present, it may be used only with robot services stopped because bus probing can interfere with active devices:

```bash
i2cdetect -y 1
```

Core should show the UPS at `36` and ADS7830 at `48`; Edge requires its UPS at `36`. Other addresses belong to other robot hardware and must not be reconfigured during this check.

## Package diagnostics

Run the installed read-only UPS diagnostic for the appropriate host:

```bash
python3 -m savo_power.diagnostics.ups_check \
  --source core_ups \
  --bus 1 \
  --address 0x36 \
  --samples 5
```

On Edge, change only `--source` to `edge_ups`. On Core, the full package launch may check UPS and ADS7830 together:

```bash
ros2 launch savo_power power_diag.launch.py role:=core
```

Do not use that unmodified full diagnostic on Edge as an acceptance check: its shared I2C check also expects the Core-only ADS7830.

## Safe-idle runtime verification

Start the appropriate role in safe idle, then inspect without publishing:

```bash
ros2 topic echo /savo_power/core/ups --once
ros2 topic echo /savo_power/edge/ups --once
ros2 topic echo /savo_power/status --once
ros2 topic echo /savo_power/health --once
```

Use the local role's UPS topic; aggregated status/health is Core-owned. Core base battery publishes `/savo_power/base/battery`. Expected result is fresh, finite, plausible readings with correct source and no I2C error.

## Interpretation and shutdown safety

Keep these distinct:

- Raw register/voltage proves only a successful read and conversion.
- UPS capacity is the HAT-reported estimate and is not established as calibrated by current repository evidence.
- Base-battery SoC is a linear estimate derived from configured voltage endpoints, not a validated fuel-gauge percentage.
- A numerically valid percentage (`0..100`) is not proof of calibrated accuracy.
- `/savo_power/shutdown_request` is passive because `automatic_shutdown_enabled=false` in Core, Edge, and diagnostics profiles.

Do not enable automatic shutdown until threshold, debounce, host shutdown, restart, and power-loss behavior pass formal hardware validation.

## Failure handling and evidence

For missing `/dev/i2c-1` or denial, correct the host I2C policy without broad permissions. For missing `0x36`, power down safely and inspect wiring/stacking rather than scanning during runtime. For implausible or invalid data, preserve raw/converted output and do not claim battery percentage or shutdown readiness. For stale ROS data, inspect the single role owner and journal.

Retain I2C device metadata, diagnostic samples, topics, power state, configuration revision, and comparison to a calibrated external instrument during later testing. Continue with the [commissioning checklist](commissioning_checklist.md) and [hardware measurement checklist](../hardware/measurement_checklist.md).
