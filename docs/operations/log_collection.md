# Log Collection

## Purpose and classification

Read-only maintainer procedure for preserving useful evidence before restart,
restore, rollback, or hardware disturbance. Do not clear logs first.

Create an access-controlled incident directory on approved storage and record
UTC/local time, timezone, hostname, operator, hardware revision, geometry
digest, Core/Edge commits, role mode/profile, and active map/location releases.

## Service and ROS evidence

Run locally on each host, substituting the applicable role unit:

```bash
systemctl status savo_core.service --no-pager
journalctl -u savo_core.service -b --no-pager
journalctl -u savo_core.service --since "<UTC-start>" --until "<UTC-end>"
git rev-parse HEAD
ros2 node list
ros2 topic list --types
ros2 service list --types
ros2 action list --types
```

Collect current operational state:

```bash
ros2 topic echo --once /savo_bringup/core/state
ros2 topic echo --once /savo_bringup/edge/state
ros2 topic echo --once /savo_control/mode_state
ros2 topic echo --once /savo_perception/safety_state
ros2 topic echo --once /savo_localization/health
ros2 topic echo --once /savo_supervisor/state_summary
ros2 topic echo --once /savo_power/health
ros2 topic echo --once /savo_nav/readiness
```

Capture publisher ownership with `ros2 topic info -v <topic>` for command,
safety, odometry, TF, scan, and incident-specific topics. Do not publish test
messages during evidence collection.

## Host, network, hardware, and storage

```bash
bash deploy/common/robot_savo_network_summary.sh
chronyc tracking
chronyc sources -v
ip -brief address
df -h /var/lib/robot_savo /var/log/robot_savo
sudo dmesg --ctime
sudo bash deploy/common/preflight_storage_check.sh
```

Core ROS/package logs are under `/var/log/robot_savo`; deploy logs are under
`${SAVO_ROOT}/log/deploy`. Edge ROS logs default to
`${HOME}/.ros/log/robot_savo_edge`. Preserve relevant map session/release
manifests, quality/verification reports, release journals, and location events.
For a staged-update failure also preserve the exact
`savo_ws/.releases/<role>-<timestamp>-<pid>/build` test results and install
identity. Ordinary workspace `savo_ws/log` and package build test results are
developer evidence, not operator cleanup targets.

For bridge/speech incidents record metadata, not secrets:

```bash
ls -ld /run/savo_bridge /run/savomind
ls -l /run/savo_bridge/command.sock /run/savo_bridge/snapshot.json
ls -l /run/savomind/speech.sock
ros2 topic echo --once /savo_bridge/readiness
ros2 topic echo --once /savo_speech/readiness
```

## Incident bundle checklist

- [ ] Source/install/hardware/geometry and release identities
- [ ] Core and Edge journals with time bounds
- [ ] ROS graph, topic owners, readiness, diagnostics, supervisor and power
- [ ] Network/Chrony and kernel/device evidence
- [ ] Map/navigation/location artifacts relevant to the event
- [ ] Bridge/speech state without credentials or unnecessary conversation data
- [ ] Photos/video and a ROS bag only when safely captured under an approved
  developer evidence plan
- [ ] File hashes, collector/operator, collection time, and access location

Never collect API tokens, credentials, full private environment files, or raw
private speech unless specifically authorized and necessary. Do not expose the
observer dashboard or Unix sockets while gathering evidence.
