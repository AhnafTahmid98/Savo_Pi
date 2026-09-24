# Temporary Core LiDAR Mapping Degraded Profile Design

## Purpose

Add an explicitly selected, temporary Core-only LiDAR autonomous-mapping
profile for supervised real-robot testing while the ToF breadboard wiring is
unreliable. The existing production profile remains the default and retains
its current fail-closed behavior.

## Profile Selection

`savo_bringup autonomous_mapping.launch.py` exposes one selector:
`autonomous_mapping_profile`. Its only accepted values are `production` and
`core_lidar_mapping_degraded`; every other value fails before nodes start.

The selector owns an atomic, allowlisted pair of assets:

| Profile | Perception configuration | Nav2 configuration |
| --- | --- | --- |
| `production` | existing `core_real_robot_v1.yaml` | existing `nav2_live_mapping.yaml` |
| `core_lidar_mapping_degraded` | new degraded Core perception profile | new degraded live-mapping Nav2 profile |

Direct perception-file and Nav2-file overrides are rejected so operators
cannot construct a mixed safety configuration. The dedicated production
runner explicitly passes `autonomous_mapping_profile:=production` and rejects
attempts to override the selector or either paired file.

## Perception and Safety Behavior

Production is unchanged: `tof_left` and `tof_right` remain required;
`depth_front` and `ultrasonic_front` remain optional; invalid or stale required
ToF data keeps range health unhealthy and the safety policy fail-closed.

Only `core_lidar_mapping_degraded` has no required range sensors. It lists
`tof_left`, `tof_right`, `depth_front`, and `ultrasonic_front` as optional. The
ToF driver remains enabled, so real values, invalid values, and driver status
remain visible on the existing range, ToF status, range health, and sensor
status topics. NaN, infinity, zero, and other invalid values remain invalid and
are reported as per-sensor errors; optionality only removes them from required
health and `required_sensor_invalid`/`required_sensor_stale` decisions.

ROS 2 Jazzy cannot infer the type of an empty YAML sequence. The degraded
profile therefore transports the empty required-sensor policy as the typed
string array `["__none__"]`. Both C++ and Python policy nodes validate and
remove that sentinel before building their effective policy or diagnostics;
mixing it with a real sensor name is rejected. The sentinel is configuration
transport only and is never treated or reported as a sensor.

The first degraded test launch keeps `perception_use_ultrasonic:=false`.
Ultrasonic support remains installed and optional. D435 depth remains optional
and is not added to Nav2.

All physical stop/slow thresholds, freshness/rate thresholds, TCA address
`0x70`, VL53L1X address `0x29`, left channel `7`, and right channel `3` remain
unchanged. No synthetic range source is introduced.

## Navigation and Motion Routing

The degraded Nav2 configuration is an exact copy of the current live-mapping
configuration except for:

- `max_vel_theta: 0.30`
- `acc_lim_theta: 0.50`
- `decel_lim_theta: -0.50`
- appending the DWB `Twirling` critic
- `Twirling.scale: 10.0`

Linear limits, footprints, costmaps, inflation, and both LiDAR obstacle layers
remain byte-for-value equivalent to production. RPLIDAR remains the SLAM and
Nav2 obstacle source. LiDAR, SLAM, localization, Nav2, base/control, and power
requirements are not weakened.

The degraded-only `Twirling` cost penalizes pure angular motion while the
holonomic base is traveling toward a frontier. It does not impose a nonzero
minimum translational velocity, remove `RotateToGoal`, or disable simultaneous
linear and angular commands. Production DWB behavior remains unchanged.

Velocity routing remains `Nav2 /cmd_vel_nav` through the existing control path
and perception safety gate to `/cmd_vel_safe`, then to `savo_base`. Nothing
under `savo_ws/src/core/savo_base/` changes.

### Stationary startup contract

For both profile selections, dedicated autonomous mapping fixes
`initial_scan360_required` to false. Its launch validates that setting before
starting nodes, hardcodes false at the child mapping boundary, and the
production runner rejects attempts to override it. Launch starts in `STOP` and
dispatches no Spin action, navigation goal, velocity command, or physical base
motion. Mission admission and map-frame start-pose capture remain stationary;
SLAM initializes without a motion prerequisite.

After mapping-local authority is acquired, the orchestrator selects NAV and
enables frontier exploration. The explorer evaluates the current robot TF pose,
selects a reachable frontier, and only then publishes the selected goal through
the guarded exploration handoff to Nav2. That frontier goal is the first source
of navigation motion. Nav2 remains free to command angular velocity when the
planned path requires it; the degraded nonzero yaw limits remain `0.30`, `0.50`,
and `-0.50` for maximum velocity, acceleration, and deceleration respectively.

## Diagnostics and Error Handling

An invalid optional ToF remains `ERROR` with its real error cause in the
per-sensor diagnostic payload. It is retained in the list of observed invalid
sensors, but it does not make the aggregate required-range status unhealthy
when the degraded profile has no required range sensors. Invalid production
ToFs continue to make the aggregate unhealthy and command a safety stop.

The Python fallback must timestamp a received invalid value at receipt time so
it is classified as `ERROR`, not falsely as never-received/stale. Initial
missing required sensors remain stale/fail-closed.

## Verification

Tests cover profile contents, production/degraded fusion decisions, aggregate
range-health behavior, non-finite input rejection, absence of synthetic data,
the allowlisted selector, rejection of mixed pairs, runner pinning, unchanged
LiDAR requirements and velocity routing, and an exact recursive Nav2 diff.
They also lock fixed-false startup Scan360, inert launch/admission behavior,
stationary pose capture, current-pose frontier planning, frontier-only initial
goal handoff, and preservation of ordinary Nav2 path-following rotation.
The recursive Nav2 guard also permits only the degraded `Twirling` critic and
scale in addition to the three approved yaw-limit differences.

Run focused tests for `savo_perception`, `savo_bringup`, `savo_nav`, and
`savo_mapping`, then the complete hardware-free workspace suite. Also run
lint/static checks, `git diff --check`, `git status --short`, and prove
`git diff -- savo_ws/src/core/savo_base` is empty. Do not commit or push.
