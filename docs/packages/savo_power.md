# savo_power

## Purpose

Role-specific UPS/base-battery acquisition, aggregate power state, health, presentation, and passive shutdown request.

## Deployment

Built on both Pis. Core runs Core UPS + base ADS7830 + aggregator/health/dashboard; Edge runs Edge UPS + health/dashboard.

## Responsibilities

Read UPS voltage/percentage, base battery ADC, validity/calibration state; aggregate sources; publish stale/fault/low-power state and optional shutdown request.

## Non-responsibilities and authority boundaries

Does not execute OS shutdown, drive motors, or assume percentage is valid without calibration. Automatic shutdown defaults disabled.

## Package structure

Production C++ drivers/nodes and equivalent Python fallback executables, role launches, YAML, tools/tests.

## Runtime components

### Core

`core_ups_node`, `base_battery_node`, `power_aggregator_node`, `power_health_node`, `power_dashboard_node`.

### Edge

`edge_ups_node`, `power_health_node`, `power_dashboard_node`. `_py` variants are retained fallbacks; C++ is production default.

## Runtime data flow

`UPS/ADS7830 -> role topics -> aggregate/health -> UI/supervisor/deployment shutdown consumer`.

## ROS interfaces

### Published topics

`/savo_power/core/ups`, `/edge/ups`, `/base/battery`, `/status`, `/health`, `/dashboard`, `/dashboard_text` (String-based structured status) and `/shutdown_request` (`Bool`).

### Subscribed topics

Aggregator/health/dashboard subscribe to the role source/status topics.

### Services

No public service.

### Actions

No action.

## TF ownership

None.

## Parameters and configuration

| Parameter | Default | Purpose |
| --- | ---: | --- |
| I2C bus/UPS address | `1/0x36` | UPS HAT |
| ADS7830 address/channel | `0x48/2` | Base battery |
| base empty/low/full | `6.40/7.20/8.40 V` | Provisional policy/calibration |
| UPS low/critical | `3.40/3.20 V` | Cell thresholds |
| stale timeout | `5 s` | Health |
| automatic shutdown | `false` | Must be validated before enabling |

## Launch files

`power_bringup` selects role; `power_core`/`power_edge` are production; `power_diag`/`power_dryrun` are tests.

## Persistent state and runtime files

No database; calibration is configuration.

## Hardware ownership

Core/Edge UPS HAT monitors; Core ADS7830 base-battery ADC. Exact electrical calibration remains hardware-specific.

## Dependencies

### Internal Robot Savo dependencies

Consumed by supervisor, bringup, UI, observer, and privileged deployment shutdown handling.

### External ROS/system dependencies

Linux I2C, ROS std/diagnostic messages.

## Safety behavior

Stale/invalid readings become unhealthy, not plausible percentages. Shutdown is a request only and disabled by default.

## Failure and degraded behavior

Individual source loss is represented in aggregate validity; missing required source closes role power readiness.

## Startup and shutdown behavior

Probes configured devices, publishes validity, releases I2C on exit.

## Build

Use Core/Edge role build.

## Run

`ros2 launch savo_power power_bringup.launch.py host_role:=core|edge`.

## Validation and testing

Tests cover formulas, aggregation, health/policy, topics, launch, static C++ and Python fallbacks.

## Current validation status

Implemented/source-tested; live readings, calibration validity, low/critical policy, fault, and shutdown integration require hardware validation.

## Known limitations and remaining validation

Voltage-to-percentage values are calibration-dependent; do not treat them as locked battery characterization.

## Change-control considerations

Addresses/channels, calibration curves, thresholds, expected-source policy, and shutdown enablement require electrical/operational review.

## Related documentation

- [Implementation README](../../savo_ws/src/shared/savo_power/README.md)
- [Power architecture](../hardware/power_architecture.md)
- [UPS setup](../setup/ups_hat_setup.md)
- [Power test plan](../testing/power_test_plan.md)
- [Ownership matrix](package_ownership_matrix.md)
