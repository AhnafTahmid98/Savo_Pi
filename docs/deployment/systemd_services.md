# Systemd Services

## Purpose

Robot Savo uses systemd to start approved production roles with deterministic environment, restart behavior, ownership, and fail-closed defaults. Service installation is a deployment step, not a substitute for role build, target validation, or motion authorization.

## Service inventory

| Unit | Intended owner | Purpose |
| --- | --- | --- |
| `savo_core.service` | Core Pi | Starts the complete Core role through `deploy/core/run_core.sh` |
| `savo_edge.service` | Edge Pi | Starts the complete Edge role through `deploy/edge/run_edge.sh` |
| `savo.service` | Either Pi | Generic role-selected wrapper using `SAVO_ROLE`; alternative to role-specific units |
| `savo_mapping.service` | Core Pi | Alternative complete Core owner for explicitly enabled manual mapping |
| `savo-location-stack@.service` | Core Pi | Typed location integration stack rendered for the selected deployment user |
| `savo-ui-runtime.service` | Edge Pi | Sole production framebuffer/UI owner, installed with the Edge service profile |
| `savo-ui.service` | Edge Pi | Optional standalone UI service supplied by `savo_ui` |
| `savo_bridge.service` | Edge Pi | Optional standalone bridge service supplied by `savo_bridge` |
| `savo-supervisor.service` | Core Pi | Optional standalone supervisor service supplied by `savo_supervisor` |

The last four units are package-local deployment options. They must not run when the same node is already started by distributed role bringup.

`savo_edge.service` declares `Wants=` and `After=` on
`savo-ui-runtime.service` and keeps `SAVO_START_UI=false`. The role-scoped Edge
installer installs both units, so the systemd runtime is the single production
UI owner. `savo-ui.service` remains an opt-in standalone alternative and must
not run with either production UI path.

## One-owner rule

Each role and component must have exactly one systemd owner.

Do not enable these combinations:

- `savo_core.service` and `savo.service` with `SAVO_ROLE=core`
- `savo_edge.service` and `savo.service` with `SAVO_ROLE=edge`
- Edge bringup with `SAVO_START_BRIDGE=true` and standalone `savo_bridge.service`
- Edge bringup with `SAVO_START_UI=true` and standalone `savo-ui.service`
- `savo-ui-runtime.service` with any other UI service/launch that owns the same framebuffer or UI node graph
- Core bringup and standalone supervisor service when both start the same supervisor node
- The normal Core role and an overlapping manual mapping stack without following the mapping-service procedure

Duplicate ownership can create duplicate node names, conflicting sockets, competing file writers, inconsistent readiness, or multiple command consumers.

The full Core runners enforce this rule twice without stopping another owner:

1. `runtime_ownership.py preflight` reads active Robot Savo services and likely
   hardware-owner processes and reports conflicts.
2. Normal Core and dedicated mapping acquire the same kernel-backed
   `/run/robot-savo/core-owner.lock`. The descriptor remains inherited by the
   top-level `ros2 launch` process, so the kernel releases it on normal exit or
   crash. The file is not a PID file and its contents are never used as lock
   state; only the live kernel `flock` is authoritative. A competing runner
   fails closed.

`/run` is volatile. The Core, mapping, and generic service profiles install
`/etc/tmpfiles.d/robot-savo-core.conf`, which recreates `/run/robot-savo` at
every boot as the rendered production user/group with mode `0750`. The
installer also runs `systemd-tmpfiles --create` once so direct runners work
before the next reboot. Core-owning systemd units use the same user/group,
`RuntimeDirectory=robot-savo`, mode `0750`, and
`RuntimeDirectoryPreserve=yes`; stopping a service therefore does not remove
the shared directory before a controlled direct-owner transition. The lock
file is created at the single fixed path with mode `0640`. Failure to access
that directory or file fails closed; there is no alternate lock-path fallback.

The preflight is diagnostic and read-only. The lifetime lock closes the race
between two otherwise simultaneous starts. Neither mechanism sends ROS
commands or stops, restarts, kills, or replaces an existing owner. Plain
`Conflicts=` is intentionally not used for the full Core alternatives because
systemd would automatically stop the active unit instead of requiring the
operator to perform the controlled `STOP` transition.

## Environment file

The shared template is:

```text
deploy/systemd/robot-savo.env.example
```

Install it as:

```bash
sudo install -d -m 0750 /etc/robot-savo
sudo install -m 0640 deploy/systemd/robot-savo.env.example \
  /etc/robot-savo/robot-savo.env
sudoedit /etc/robot-savo/robot-savo.env
```

The template contains role, ROS, and feature configuration only. Do not add
`SAVO_ROOT` or `SAVO_WS` to it. `render_units.sh` generates
`robot-savo.paths.env` from the same `--root` value used for `ExecStart` and
`WorkingDirectory`; every rendered unit loads that generated file after the
optional configuration file so paths cannot disagree.

Keep these safety defaults unless a reviewed test procedure requires a controlled change:

```text
SAVO_ROBOT_MODE=safe_idle
SAVO_BRINGUP_PROFILE=lidar_only
SAVO_CONTROL_STARTUP_MODE=STOP
SAVO_START_UI=false
```

The role-specific run scripts additionally default locked-geometry enforcement to true, provisional geometry to false, and D435 voxel validation to false.

## Render service templates

Templates contain deployment placeholders and must be rendered before installation:

```bash
cd ~/Savo_Pi
sudo bash deploy/systemd/render_units.sh \
  --user "$USER" \
  --group "$(id -gn)" \
  --root "$PWD" \
  --output-dir /tmp/robot-savo-units
```

The renderer:

- Validates the supplied user, group, and absolute repository root.
- Renders the Core, Edge, generic role, mapping, location, and UI units.
- Generates a matching `robot-savo.paths.env`.
- Runs `systemd-analyze verify` when available.
- Does not install, enable, or start any service.

`deploy/core/build_core.sh --test` runs the deployment contract tests and this
render/ownership/environment validator after the selected ROS package tests. It
still performs no service installation or activation.

Review the generated files. Prefer the role-scoped installer, whose default is
render, verify, install, and daemon-reload only:

```bash
sudo bash deploy/systemd/install_services.sh \
  --profile core \
  --user "$USER" \
  --group "$(id -gn)" \
  --root "$PWD"
```

Select exactly one of `core`, `mapping`, `generic`, `edge`, or `location`.
The Edge profile installs `savo_edge.service` and `savo-ui-runtime.service` as
one ownership bundle. The installer never enables or starts a service.
Core, mapping, and generic profiles additionally install the boot-time
tmpfiles policy and create the private runtime directory; this does not start a
ROS or systemd service.

## Install a role-specific service

### Core

Use `install_services.sh --profile core`, review the installed unit and paths
environment, then enable it only as a separate explicit operator action.

### Edge

Use `install_services.sh --profile edge`. This installs both the Edge role and
its single systemd-owned UI runtime without activating either.

Enable does not require immediate start. Perform the first safe-idle launch interactively and start the service only after that check passes.

## Service hardening and lifecycle

The role units use:

- A non-root deployment user and group.
- The repository as `WorkingDirectory`.
- An optional protected environment file.
- Safe-idle and `STOP` environment defaults.
- `Restart=on-failure` with bounded restart delay.
- `SIGINT` for ROS-aware shutdown.
- Control-group termination.
- Restrictive `UMask=0027`.
- `NoNewPrivileges=true`.

The Edge unit also requires membership in `savomind-bridge` for the local speech socket contract.

## Mapping service gate

`savo_mapping.service` is intentionally fail-closed. It requires both:

```text
/etc/robot-savo/enable-mapping-service
```

and `SAVO_ENABLE_MAPPING_SERVICE=true`.

Its runner also requires:

- Control startup mode `STOP`
- Locked geometry
- Provisional geometry disabled

Create the marker only for a controlled mapping session:

```bash
sudo install -d -m 0750 /etc/robot-savo
sudo touch /etc/robot-savo/enable-mapping-service
```

Set the environment flag through the protected environment file, start the service, perform the approved session, then disable the service and remove the marker:

```bash
sudo systemctl disable --now savo_mapping.service
sudo rm -f /etc/robot-savo/enable-mapping-service
```

Do not leave autonomous or manual mapping enabled as an undocumented boot behavior.

The mapping unit is an alternative full Core owner, not an overlay on
`savo_core.service`. Use this exact transition while the robot is stationary:

```bash
ros2 run savo_control mode_cmd_cli.py STOP
sudo systemctl stop savo_core.service
systemctl is-active savo_core.service savo.service
python3 deploy/common/runtime_ownership.py preflight --owner mapping
sudo systemctl start savo_mapping.service
```

Both normal Core and mapping runners then contend for the same lifetime lock.
The systemd ownership preflight is an `ExecCondition`, so an already-owned robot
is skipped instead of entering a restart loop. A race caught later by the runner
or lifetime lock exits with ownership code `4`, which is excluded from automatic
restart. Ownership therefore never transfers later merely because the first owner
stops; a fresh operator start is required.

If preflight or lock acquisition reports an owner, do not retry around the
guard; identify and explicitly stop the correct owner. Reverse the sequence by
returning to `STOP`, stopping mapping, verifying it inactive, and then starting
the normal Core owner.

The normal production Core runner refuses `manual_mapping` and
`autonomous_mapping`. Use `deploy/core/run_mapping_service.sh` for guarded manual
mapping and `deploy/core/run_autonomous_mapping.sh` for the dedicated
Supervisor-free autonomous mapping composition. Direct ROS launch remains a
development/component interface, not a production ownership path.

## Critical child exit visibility

The Core service supervises the top-level `ros2 launch` process. The Base launch
does not mark `base_driver_node` as a launch-critical process, so after that
child exits the parent ros2 launch process may remain active and systemd may
still report `savo_core.service` as active. `Restart=on-failure` applies to the
parent, not every ROS child. Missing Base health/readiness must still fail
closed, and a latched board fault still requires explicit operator recovery.
Do not interpret systemd `active` as readiness and do not add unattended
whole-stack restart or automatic motion recovery for this case.

## Unresolved whole-bus I2C incident

The Core controller timeout remains unresolved. The ownership guard only lets
the next run prove whether duplicate software owners were absent; it is not an
I2C fix. Before the next run retain:

- active/activating `savo*` units and ownership-preflight output;
- `/proc`/process evidence for Base, IMU, UPS/ADS, VL53 mux, and mapping owners;
- timestamped kernel `i2c_designware` messages and matching ROS errors;
- cold-start bus state collected only while ROS I2C owners are stopped;
- SDA/SCL electrical state, peripheral isolation results, supply voltage, and
  load/concurrency conditions.

That evidence is required to distinguish duplicate owners, a peripheral or
electrical bus hold, kernel/controller behavior, load interaction, and power
integrity. Do not run `i2cdetect` while ROS I2C nodes own the bus.

## Start, stop, and inspect

```bash
sudo systemctl start savo_core.service
sudo systemctl stop savo_core.service
sudo systemctl restart savo_core.service
systemctl status savo_core.service --no-pager
journalctl -u savo_core.service -b --no-pager
```

Use the equivalent Edge service name on the Edge Pi.

For live logs:

```bash
journalctl -u savo_core.service -f
```

A service shown as active does not prove robot readiness. Inspect ROS readiness, control mode, supervisor state, sensors, TF, and diagnostics separately.

## Boot verification

After enabling a role service, reboot only in a safe physical configuration:

```bash
sudo reboot
```

After boot verify:

```bash
systemctl is-enabled savo_core.service
systemctl is-active savo_core.service
journalctl -u savo_core.service -b --no-pager
```

### Pass criteria

- Only the intended role owner is enabled.
- The service starts in safe idle.
- Core control remains `STOP`.
- No unintended wheel or head motion occurs.
- Runtime directories and permissions are correct.
- The service stops cleanly on `systemctl stop`.
- Restart-on-failure does not create a rapid failure loop.
- Edge local sockets remain inaccessible to unauthorized users.

## Log rotation

The repository supplies:

```text
deploy/logrotate/robot-savo
```

Review user/group assumptions, then install and test it with the target's log paths. Do not rotate or delete active map, location, supervisor, or release-state files as if they were logs.

## Change control

A service change must include:

- Source and template review.
- Rendered-unit verification.
- Environment-file compatibility review.
- Duplicate-owner analysis.
- Safe-idle boot test.
- Stop/restart test.
- Log and state-path validation.
- Rollback instructions.
