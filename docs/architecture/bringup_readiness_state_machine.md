# Bringup Readiness State Machine

`savo_bringup` publishes independent Core and Edge readiness decisions. Readiness permits a configured launch graph to operate; it is not supervisor arming or motion authorization.

## States

```text
starting
  -> waiting_for_dependencies
  -> validating_geometry
  -> waiting_for_safety
  -> waiting_for_localization
  -> waiting_for_map_context
  -> waiting_for_navigation
  -> ready
  -> degraded (ready=true; optional dependency unhealthy)

any required failure or startup timeout with missing required input -> blocked
shutdown request -> shutting_down
```

The first unmet required dependency chooses the relevant waiting state. A required explicit failure blocks immediately. A missing/stale/not-ready required dependency waits until `startup_timeout_s`, then blocks. Optional failed or unhealthy observations produce `degraded` and retain `ready=true`. Current node defaults are `2 Hz`, `45 s` startup timeout, and `3 s` freshness; role YAML may override them.

## Inputs and outputs

Depending on role/mode/profile, inputs include geometry policy, base/control/safety, LiDAR/perception, localization, power, supervisor heartbeat/authority, mapping, verified active release, map context, navigation/goal admission, bridge, RealSense, VO, speech, UI, and obstacle-cloud health.

Outputs under the configured role namespace are:

- `state`: retained string with state, ready flag, role, mode, profile, and reason;
- `ready`: retained Boolean;
- `heartbeat`: reliable counter;
- `diagnostics`: diagnostic status and missing/failed/degraded lists.

Motion-capable non-bench operation requires locked geometry by contract. `lidar_d435_voxel` requires explicit D435 validation. Safe idle and diagnostics have different dependency requirements from mapping/navigation.

Readiness never writes a motor, selects a map, approves a release, or grants supervisor permission. Consumers must combine it with operation-owned admission and current authority. Tests cover state evaluation and namespace separation; two-host stale/timeout, recovery, shutdown, and per-profile dependency injection remain integration checks.
