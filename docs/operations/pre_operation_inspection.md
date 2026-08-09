# Pre-Operation Inspection

## Purpose and classification

This operator inspection is required before each powered motion session and
after transport, impact, repair, software update, or unexplained shutdown. It
must be completed with control in `STOP`; physical checks near moving parts
require drivetrain power isolation.

## Physical and environment checks

- [ ] Mecanum wheels/rollers rotate as expected and show no loose or trapped
  parts.
- [ ] Chassis, decks, fasteners, sensor brackets, and pan/tilt assembly are
  secure.
- [ ] LiDAR sweep and all camera/range fields are unobstructed.
- [ ] Head travel is clear and its cables cannot snag.
- [ ] Wiring is insulated, strain-relieved, and outside drivetrain envelopes.
- [ ] Batteries/UPS units show no swelling, leakage, damage, or loose mounting.
- [ ] The installed emergency power/stop control is reachable.
- [ ] Floor, slopes, edges, people, reflective hazards, and clearance suit the
  approved operation.
- [ ] There is no abnormal heat, smell, vibration, or noise.

## Electrical checks

- [ ] Core, Edge, base/motor power, Ethernet, USB, I2C/GPIO harnesses, camera,
  display/audio, and approved charger connections are secure.
- [ ] Core and Edge power without reboot loops or undervoltage indications.
- [ ] Dedicated Ethernet link is present.
- [ ] No cable or connector is hot, discoloured, damaged, or intermittently
  connected.

Do not trust charge percentage until calibrated. Use power health, raw voltage,
UPS state, measurement validity, and shutdown-request state together.

## Software checks

On the applicable host, source ROS and the installed workspace, then collect:

```bash
git rev-parse HEAD
systemctl is-active savo_core.service
systemctl is-active savo_edge.service
ros2 node list
ros2 topic echo --once /savo_bringup/core/state
ros2 topic echo --once /savo_control/mode_state
ros2 topic echo --once /savo_perception/safety_state
ros2 topic echo --once /savo_localization/health
ros2 topic echo --once /savo_power/health
ros2 run tf2_ros tf2_echo odom base_footprint
```

Run only the local service check on each host. For navigation also inspect
`/savo_nav/readiness`, `/savo_nav/map_context/status`, and the read-only
`/var/lib/robot_savo/maps/production/active_map.yaml` identity. For named
navigation confirm the location release and map identity through the approved
UI/administration workflow.

## Result criteria

| Result | Meaning | Action |
| --- | --- | --- |
| PASS | All required physical, electrical, state, and release checks current | Proceed to the selected runbook |
| BLOCKED | Missing approval, measurement, optional dependency, or evidence | Keep `STOP`; resolve or choose a permitted reduced scope |
| FAIL | Damage, fault, stale safety state, wrong mode, or unsafe environment | Isolate and escalate; do not operate |

Retain the checklist, commit, hardware revision, geometry digest, mode/profile,
map/location release when applicable, and anomalies.

