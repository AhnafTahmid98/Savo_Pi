# Systemd Services

## Purpose

Robot Savo uses systemd to start approved production roles with deterministic environment, restart behavior, ownership, and fail-closed defaults. Service installation is a deployment step, not a substitute for role build, target validation, or motion authorization.

## Service inventory

| Unit | Intended owner | Purpose |
| --- | --- | --- |
| `savo_core.service` | Core Pi | Starts the complete Core role through `deploy/core/run_core.sh` |
| `savo_edge.service` | Edge Pi | Starts the complete Edge role through `deploy/edge/run_edge.sh` |
| `savo.service` | Either Pi | Generic role-selected wrapper using `SAVO_ROLE`; alternative to role-specific units |
| `savo_mapping.service` | Core Pi | Explicitly enabled manual-mapping entry point with an external marker gate |
| `savo-location-stack@.service` | Core Pi | Typed location integration stack for a named deployment user |
| `savo-ui.service` | Edge Pi | Optional standalone UI service supplied by `savo_ui` |
| `savo_bridge.service` | Edge Pi | Optional standalone bridge service supplied by `savo_bridge` |
| `savo-supervisor.service` | Core Pi | Optional standalone supervisor service supplied by `savo_supervisor` |

The last three units are package-local deployment options. They must not run when the same node is already started by distributed role bringup.

## One-owner rule

Each role and component must have exactly one systemd owner.

Do not enable these combinations:

- `savo_core.service` and `savo.service` with `SAVO_ROLE=core`
- `savo_edge.service` and `savo.service` with `SAVO_ROLE=edge`
- Edge bringup with `SAVO_START_BRIDGE=true` and standalone `savo_bridge.service`
- Edge bringup with `SAVO_START_UI=true` and standalone `savo-ui.service`
- Core bringup and standalone supervisor service when both start the same supervisor node
- The normal Core role and an overlapping manual mapping stack without following the mapping-service procedure

Duplicate ownership can create duplicate node names, conflicting sockets, competing file writers, inconsistent readiness, or multiple command consumers.

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

The template must be adapted for the target. In particular, update `SAVO_ROOT`, `SAVO_WS`, `SAVO_ROLE`, ROS domain, middleware, and feature flags.

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
  --group "$USER" \
  --root "$PWD" \
  --output-dir /tmp/robot-savo-units
```

The renderer:

- Validates the supplied user, group, and absolute repository root.
- Renders the Core, Edge, generic role, mapping, and UI units that exist.
- Runs `systemd-analyze verify` when available.
- Does not install, enable, or start any service.

Review the generated files before copying them to `/etc/systemd/system/`.

## Install a role-specific service

### Core

```bash
sudo install -m 0644 /tmp/robot-savo-units/savo_core.service \
  /etc/systemd/system/savo_core.service
sudo systemctl daemon-reload
sudo systemctl enable savo_core.service
```

### Edge

```bash
sudo install -m 0644 /tmp/robot-savo-units/savo_edge.service \
  /etc/systemd/system/savo_edge.service
sudo systemctl daemon-reload
sudo systemctl enable savo_edge.service
```

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
