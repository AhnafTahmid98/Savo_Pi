# First installation checklist

## Installation record

```text
Installation date:
Installer:
Robot hardware revision:
Core hostname:
Edge hostname:
Core OS version:
Edge OS version:
ROS distribution:
Repository commit:
Network configuration:
ROS domain:
Middleware:
Geometry state:
```

Record source output with `git rev-parse HEAD`; do not write credentials into this record.

## Hardware

- [ ] The physical Core and Edge Raspberry Pi 5 units are identified and labelled.
- [ ] Storage media and approved power/UPS modules are installed.
- [ ] The dedicated Ethernet link is connected.
- [ ] Sensors, display, audio, and robot wiring match the [wiring overview](../hardware/wiring_overview.md) and [GPIO/I2C map](../hardware/gpio_i2c_map.md).
- [ ] An emergency stop is present and accessible.
- [ ] The robot is secured against unintended motion; wheels are clear or mechanically restrained as approved.

## OS

- [ ] Ubuntu 24.04 LTS ARM64 is installed on both Pis.
- [ ] A named, non-root deployment user exists on each host.
- [ ] Hostnames are `core`/`savo-core` and `edge`/`savo-edge` respectively.
- [ ] SSH access, if approved, is key-based and a local-console recovery path exists before network changes.
- [ ] Locale, timezone, and package updates are recorded.

Verify with:

```bash
hostnamectl
uname -m
grep '^VERSION_ID=' /etc/os-release
timedatectl
```

## Repository and ROS

- [ ] Git access is configured without storing private keys in the repository.
- [ ] The authorized repository is checked out at `$HOME/Savo_Pi`, or the configured absolute root.
- [ ] The revision and dirty state are recorded.
- [ ] ROS 2 Jazzy is installed under `/opt/ros/jazzy` and `ros2` starts.

```bash
cd "$HOME/Savo_Pi"
git status --short
git rev-parse HEAD
git diff --check
source /opt/ros/jazzy/setup.bash
ros2 --help
```

## Role dependencies

- [ ] Core: `bash deploy/common/install_role_deps.sh --role core` passes on the Core host.
- [ ] Edge: `bash deploy/common/install_role_deps.sh --role edge` passes on the Edge host.
- [ ] Observer dependencies resolve and `bash deploy/observer/build_observer.sh` passes on the workstation.
- [ ] Installer logs under the workspace log area are retained.

## Network and time

- [ ] Actual Ethernet and upstream interfaces are identified before rendering configuration.
- [ ] The dedicated addresses and subnet are recorded.
- [ ] Core–Edge ping succeeds in both directions.
- [ ] Upstream/DNS access works where required.
- [ ] Chrony is active and the Edge selects Core or a documented fallback source.
- [ ] Core, Edge, and observer use the same `ROS_DOMAIN_ID` and compatible RMW.

## Permissions

- [ ] Core serial/LiDAR access is verified; source explicitly expects `dialout` for serial access.
- [ ] I2C and GPIO access is verified under the installed host policy.
- [ ] Edge RealSense/video and audio access is verified.
- [ ] Framebuffer and touch access is verified only if UI is enabled.
- [ ] No device or runtime directory is world-writable.
- [ ] Any host-local permission policy is recorded because this repository ships no general hardware udev rules.

## Storage and runtime paths

- [ ] `/var/lib/robot_savo` and `/var/log/robot_savo` were created with the Core storage script.
- [ ] Ownership is the selected Core runtime user/group and mode is `0750`.
- [ ] `/run/savomind` is prepared by the Edge socket script and is recreated by tmpfiles.
- [ ] The selected bridge ownership model creates `/run/savo_bridge`; directory preparation is not mistaken for socket creation.
- [ ] Persistent data is not stored under `/tmp` or `/run`.

## Build

- [ ] Core `bash deploy/core/build_core.sh --clean --test` reports all 14 packages and zero test failures/errors.
- [ ] Edge `bash deploy/edge/build_edge.sh --clean --test` reports all 10 packages and zero test failures/errors.
- [ ] Observer build and `bash deploy/observer/validate_observer.sh` pass.
- [ ] No missing-package development bypass was used as release evidence.

## Commissioning

- [ ] Hardware is detected and accessible without an actuating command.
- [ ] Core and Edge start interactively in `safe_idle` with control `STOP`.
- [ ] Geometry lock remains enforced and D435 obstacle-cloud validation remains false.
- [ ] Cross-host ROS discovery and clock agreement are verified.
- [ ] There is one owner for each role, bridge, UI, and sensor.
- [ ] No unintended movement or motor output occurs.
- [ ] The [commissioning checklist](commissioning_checklist.md) is marked `PASS`, `BLOCKED`, or `FAIL` with evidence.

## Failure handling and evidence

Stop at the first failed prerequisite. Preserve the command, exit code, logs, host, time, revision, and configuration; fix the cause without weakening permissions or safety gates. A completed checklist does not replace the [formal test plan](../testing/full_robot_test_plan.md).
