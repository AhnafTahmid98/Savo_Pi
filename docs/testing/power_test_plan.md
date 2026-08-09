# Power Test Plan

## Objective

Verify honest Core/Edge power acquisition, aggregation, validity/calibration state, stale/fault handling, passive shutdown requests, and recovery without unsafe battery discharge.

## Scope

Core UPS, Edge UPS, Core ADS7830/base battery, aggregator/health/dashboard, and shutdown request. Privileged OS shutdown execution is deployment-owned.

## Test ownership

Power maintainer owns PWR-001–003; qualified electrical/hardware operator owns PWR-004–008.

## Safety classification

PWR-001–003 are `STATIC`/`UNIT`/`PC`; PWR-004–008 are `HARDWARE-NON-ACTUATING`, `INTEGRATION`, `FAULT-INJECTION`, `PERSISTENT-STATE`, or `POWER-DISRUPTION`. Never deep-discharge a battery to create a case.

## Preconditions

Approved wiring, undamaged charged batteries, calibrated meter, current device addresses/channels, control STOP, automatic shutdown disabled unless a separate approved injection test explicitly enables it.

## Required hardware

Core/Edge UPS HATs, Core ADS7830/base battery monitor, meter, current-limited supply or safe simulator where available, and proper PPE/isolation.

## Required software / configuration

Role profiles, UPS/base/aggregator/health YAML, provisional calibration identified, and deployment shutdown consumer policy.

## Interfaces under test

`/savo_power/{core/ups,edge/ups,base/battery,status,health,dashboard,dashboard_text,shutdown_request}`. No service/action or OS-shutdown execution is owned.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| PWR-001 | T0 `STATIC` | Verify role graph, I2C bus/address, ADS7830 channel, validity fields, stale/low/critical policy, provisional calibration and `automatic_shutdown=false`. |
| PWR-002 | T1 `UNIT`/`PC` | Build/tests pass for formulas, aggregation, expected sources, validity/calibration, health, stale/invalid policy, launch and C++/fallback contracts. |
| PWR-003 | T2 `TARGET-NON-HARDWARE` | Dry-run/safe injection proves missing/invalid/NaN/out-of-range readings are unhealthy and never converted to a confident percentage; low-power can issue a request without executing shutdown. |
| PWR-004 | T3 `HARDWARE-NON-ACTUATING` | On Core and Edge separately, detect intended devices, record raw voltage/ADC/validity and compare with meter under stable safe conditions. |
| PWR-005 | T4 `HARDWARE-NON-ACTUATING` | Characterize configured calibration at safe operating points; until approved, percentage remains explicitly uncalibrated/provisional. Do not induce deep discharge. |
| PWR-006 | T5 `INTEGRATION` | Aggregate Core sources and Edge source; supervisor/UI/bringup show correct role, validity, stale and low-power state; missing one source cannot be masked by another. |
| PWR-007 | T6 `FAULT-INJECTION` | Using supported simulation or safe device disconnect, test stale source, invalid ADC, disconnect, and low/critical policy. Shutdown is only a request and remains bounded/observable. |
| PWR-008 | T7 `RECOVERY` | Reconnect/restart nodes and role; validity returns only with fresh plausible readings, no stale percentage is replayed, and repeated faults are diagnostic. |

## Pass criteria

Raw measurements agree with approved tolerance when established; calibration state is truthful; aggregation exposes missing sources; stale/invalid data is unhealthy; shutdown remains a request; recovery needs fresh data.

## Blocked criteria

Unsafe battery state, missing meter/device/access, calibration/threshold acceptance not established, or no safe injection path.

## Failure criteria

False-valid percentage, masked source loss, stale data shown healthy, wrong role/device/channel, unintended OS shutdown, unsafe discharge, or recovery from old data.

## Abort criteria

Isolate power immediately on swelling, smoke, heat, smell, arcing, wrong polarity, unexpected voltage/current, damaged cell, or unstable supply.

## Evidence to retain

Device/address/role, meter calibration and paired readings, raw/derived/validity samples, config, temperature/current where available, injection method, shutdown-request and recovery timelines, photos/logs, reviewer.

## Regression triggers

UPS/ADC hardware/address/channel, divider/calibration/thresholds, expected sources, aggregation/stale policy, shutdown enablement/consumer, wiring or power topology.

## Current validation status

Source-tested; current live calibration, thresholds, device faults, aggregate and shutdown integration are hardware-dependent and `NOT RUN` by Phase 7.

## Related documentation

- [Power package](../packages/savo_power.md)
- [Power architecture](../hardware/power_architecture.md)
- [UPS setup](../setup/ups_hat_setup.md)

