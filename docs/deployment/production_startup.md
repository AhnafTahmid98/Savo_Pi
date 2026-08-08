# Production Startup

## Purpose

This procedure starts the distributed Robot Savo system in a controlled, observable, fail-closed state. It assumes Core and Edge have already passed dependency installation, clean role builds, tests, service rendering, and component setup.

Production startup is not the moment to discover unknown geometry, unresolved device permissions, duplicate services, or missing persistent directories.

## Required preconditions

Before every production start:

- The robot is in a physically safe area.
- The emergency stop is reachable.
- The tested Core and Edge source revisions are recorded and compatible.
- No unreviewed source or configuration changes are present.
- Persistent state has a recent backup.
- Core and Edge clocks and network link are healthy.
- Active map and semantic-location release identities are known when navigation is planned.
- Locked geometry is available for any motion-capable mode.
- The D435 obstacle cloud remains disabled unless its hardware gate has passed.
- Only one service owns each role and optional component.

## Startup order

Use this order:

1. Inspect the physical robot and power system.
2. Start the Core role in safe idle.
3. Verify Core readiness and `STOP`.
4. Start the Edge role in safe idle.
5. Verify Core–Edge discovery and Edge health.
6. Start SavoMind only after its local socket and authority boundary are ready.
7. Start the observer workstation.
8. Enable only the mission-specific capabilities required for the approved session.
9. Authorize motion only through the applicable test or operating procedure.

## 1. Physical inspection

Check:

- Wheels and mecanum rollers are intact.
- No cable can enter the drivetrain.
- LiDAR rotates freely.
- Head pan/tilt path is unobstructed.
- Camera, ToF, ultrasonic, IMU, and RealSense mounts are secure.
- Batteries and UPS modules show no swelling, overheating, or damage.
- The work area has a clear stop zone.
- The operator understands the abort signal and emergency-stop action.

Any mechanical or power anomaly blocks startup.

## 2. Verify services and ownership

On Core:

```bash
systemctl list-unit-files 'savo*' --no-pager
systemctl --type=service --state=running 'savo*' --no-pager
```

On Edge, run the same commands.

Confirm there is no duplicate role, bridge, UI, supervisor, mapping, or location stack.

## 3. Start Core

With systemd:

```bash
sudo systemctl start savo_core.service
systemctl status savo_core.service --no-pager
```

For an interactive maintenance start:

```bash
cd ~/Savo_Pi
bash deploy/core/run_core.sh
```

Verify immediately:

- Launch reports `role=core`.
- Robot mode is `safe_idle`.
- Bringup profile is the approved profile.
- Startup mode is `STOP`.
- Required runtime directories are writable.
- Supervisor starts without an unresolved persistent-state error.
- No drivetrain output occurs.

If locked geometry is required but unavailable, the system must remain blocked. Do not set `SAVO_ALLOW_PROVISIONAL_GEOMETRY=true` for production startup.

## 4. Verify Core ROS state

In a separate shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Savo_Pi/savo_ws/install/setup.bash
ros2 node list
ros2 topic list
```

Then use the package dashboards, diagnostics, and readiness topics defined by the current package documentation. At minimum verify:

- Core bringup state and heartbeat are current.
- Supervisor reports a safe, non-armed startup condition.
- Control mode is `STOP`.
- Perception safety input state is current or fails closed.
- Base watchdog is active.
- TF does not have duplicate publishers.
- No unexpected command topic is active.

## 5. Start Edge

With systemd:

```bash
sudo systemctl start savo_edge.service
systemctl status savo_edge.service --no-pager
```

For an interactive maintenance start:

```bash
cd ~/Savo_Pi
bash deploy/edge/run_edge.sh
```

Verify:

- Launch reports `role=edge`.
- Edge mode/profile match Core.
- RealSense and VO start only through the configured owners.
- Obstacle cloud remains off unless approved.
- Speech and UI remain off unless approved.
- `/run/savo_bridge` exists with the approved owner/group/mode when distributed bringup owns the bridge.
- The bridge starts once and can publish its snapshot.
- No Edge component publishes directly to motor execution.

## 6. Verify network and time

From Core and Edge:

```bash
ip -brief address
chronyc tracking
chronyc sources -v
```

Verify the dedicated link, ROS domain, middleware, and clock agreement. Then confirm distributed ROS discovery from both hosts.

Large or unstable clock offsets block VO, TF, sensor fusion, and reliable correlation of speech or mission events.

## 7. Start SavoMind

SavoMind is a separate companion deployment. Start it only after:

- `/run/savomind` exists with the approved group and mode.
- The speech socket contract version matches.
- The SavoMind runtime user/container has only the required group access.
- Provider credentials are loaded through protected configuration.
- The ROS bridge command boundary is already available when command integration is required.

SavoMind may perform STT, LLM planning, and TTS, but it does not own motors, mapping release, navigation readiness, supervisor authority, or operator approval.

## 8. Start observer tools

On the operator workstation:

```bash
cd ~/Savo_Pi
bash deploy/observer/validate_observer.sh
```

Build/source the observer workspace as documented, then launch the read-only dashboard or RViz profile. Verify that observation does not create command publishers or services.

## 9. Select the approved operating mode

The production run wrappers default to `safe_idle` and `lidar_only`. Mode/profile changes must be explicit, documented, and supported by the current geometry and hardware-validation state.

| Activity | Minimum prerequisites |
| --- | --- |
| Diagnostics | Safe idle, no moving diagnostic unless explicitly authorized |
| Manual drive | Locked geometry, safety sensors, control/base tests, operator at emergency stop |
| Manual mapping | Manual-drive gate plus LiDAR/localization/mapping setup |
| Autonomous mapping | Manual map evidence, mission authority, quality/review workflow, recovery validation |
| Saved-map navigation | Approved AM-8 production release, localization, safety, Nav2, named-location validation |
| Speech interaction | Edge audio and protocol-v2 round-trip validation |
| D435 voxel profile | Independent D435 obstacle-cloud hardware validation |

## Shutdown

Return motion authority to `STOP` before stopping services. Then shut down in reverse order:

1. Stop active mission and cancel goals.
2. Confirm zero safe velocity and drivetrain stop.
3. Stop SavoMind and optional Edge components.
4. Stop Edge role.
5. Stop Core role.
6. Verify maps, locations, and supervisor state are flushed.
7. Power down through the approved power procedure.

Example:

```bash
sudo systemctl stop savo_edge.service
sudo systemctl stop savo_core.service
```

Review the journals for shutdown errors before removing power.

## Startup acceptance record

Record:

- Date/time:
- Core commit:
- Edge commit:
- Core service owner:
- Edge service owner:
- Robot mode/profile:
- Geometry profile and digest:
- Active map release:
- Location release:
- Core readiness:
- Edge readiness:
- Control mode:
- Supervisor state:
- Network/time status:
- Optional features enabled:
- Operator and emergency-stop observer:
- Decision: PASS / BLOCKED / FAIL
