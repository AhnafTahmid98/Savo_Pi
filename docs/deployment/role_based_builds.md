# Role-Based Builds

## Purpose

Robot Savo uses one ROS 2 workspace on multiple computers, but each computer
builds only the packages assigned to its deployment role. The Core and Edge
role scripts are the supported target build entry points.

The authoritative package arrays are defined in:

- `deploy/core/env_core.sh` — `SAVO_CORE_BUILD_PACKAGES`
- `deploy/edge/env_edge.sh` — `SAVO_EDGE_BUILD_PACKAGES`

Do not maintain an independent package list in documentation, CI, or local
helpers. Change role membership in the applicable environment file, then run
the role validators and build. The [package ownership matrix](../packages/package_ownership_matrix.md)
is the human-readable snapshot of those arrays.

## Supported targets

| Role | Host constraint | Script under `deploy/` | Count |
| --- | --- | --- | ---: |
| Core | `core` or `savo-core` | `core/build_core.sh` | 14 |
| Edge | `edge` or `savo-edge` | `edge/build_edge.sh` | 10 |
| Observer | Workstation; no check | `observer/build_observer.sh` | Separate |

Core and Edge hostname checks are intentional safeguards against running
hardware-facing target builds on the wrong computer. The observer build is a
workstation workflow and builds through `--packages-up-to savo_observer`.

## Prerequisites

Before a target role build:

1. Use Ubuntu 24.04 and ROS 2 Jazzy.
2. Place the repository at the configured `SAVO_ROOT` and workspace at
   `SAVO_WS`.
3. Confirm at least 8 GiB of free space for dependency installation or staged
   update.
4. Confirm every package required by the selected role is present.
5. Stop manual ROS sessions that source a conflicting overlay.
6. Install dependencies and perform the installer-managed initial role build.

For Core:

```bash
cd ~/Savo_Pi
bash deploy/common/install_role_deps.sh --role core
```

For Edge:

```bash
cd ~/Savo_Pi
bash deploy/common/install_role_deps.sh --role edge
```

`install_role_deps.sh`:

- requires Ubuntu 24.04;
- requires ARM64 unless `SAVO_ALLOW_PC_INSTALL=true` is deliberately set on a
  development computer;
- reads the selected role array from `env_core.sh` or `env_edge.sh`;
- installs common and role-specific operating-system packages;
- initializes or updates `rosdep` and installs dependencies only for selected
  role package paths;
- runs a `colcon build --packages-select ... --symlink-install` for the role;
- prepares Edge runtime socket directories after a successful Edge build; and
- writes an install log under `${SAVO_DEPLOY_LOG_DIR}` or, by default,
  `${SAVO_ROOT}/log/deploy`.

Use `--dry-run` to print the selected APT packages, role packages, and planned
ROS dependency/build scope without installing or building.

## Core build

Run on the Core Pi:

```bash
cd ~/Savo_Pi
bash deploy/core/build_core.sh --clean --test
```

The script consumes `SAVO_CORE_BUILD_PACKAGES` directly.

| Option | Behavior |
| --- | --- |
| `--clean` | Removes `savo_ws/build`, `install`, and `log` before building |
| `--test` or `--tests` | Runs tests and prints the verbose result summary |
| `--allow-missing` | Omits missing packages for staged development only |

Never use `--allow-missing` for production or release qualification. Without
that option, one missing Core package fails the build before `colcon build`.

## Edge build

Run on the Edge Pi:

```bash
cd ~/Savo_Pi
bash deploy/edge/build_edge.sh --clean --test
```

The script consumes `SAVO_EDGE_BUILD_PACKAGES` directly. Every required Edge
package must exist; the script has no `--allow-missing` mode.

## Build behavior

Both target scripts:

- source the configured ROS 2 distribution;
- enforce the applicable Core or Edge short hostname;
- inspect the workspace with `colcon list`;
- build only the authoritative role array with `--symlink-install`;
- source the completed role overlay before requested tests; and
- return non-zero for unknown arguments, missing prerequisites or packages,
  build failures, and test failures.

The scripts do not launch Robot Savo, install or enable systemd units, apply
network configuration, arm the supervisor, or authorize motion.

## Pass criteria

A target role build passes only when:

1. The hostname check succeeds.
2. Every required role package is present.
3. `colcon build` exits successfully.
4. When tests are requested, `colcon test` exits successfully and
   `colcon test-result --verbose` reports zero errors and failures.
5. No package was omitted through `--allow-missing`.
6. No safety gate or production default was weakened to obtain the result.

Retain the terminal output together with the source revision, hostname, date,
timezone, and selected role.

## Failure handling

If a build fails:

1. Do not launch from the partial install.
2. Record the first build or test failure, not only the final summary.
3. Correct the dependency or source issue in the repository.
4. Rerun the target build with `--clean --test`.
5. Do not use `--allow-missing` to qualify an incomplete Core build.
6. Do not copy another host's `build`, `install`, or `log` trees as a
   substitute for a target build.

## Staged target updates

For an already deployed target, use the staged updater instead of overwriting
the active install in place:

```bash
cd ~/Savo_Pi
bash deploy/common/update_role.sh --role core --pull
```

or:

```bash
cd ~/Savo_Pi
bash deploy/common/update_role.sh --role edge --pull
```

The updater:

- requires a Git checkout, ROS workspace, `colcon`, and at least 8 GiB free;
- rejects a dirty checkout by default;
- uses `git fetch` plus a fast-forward-only merge for `--ref`, or
  `git pull --ff-only` otherwise;
- reads the same authoritative role arrays as normal builds;
- verifies the entire role package set before stopping an active service;
- stages build, install, and tests under `savo_ws/.releases/`;
- activates the staged install only after build and tests succeed;
- retains the prior install as `savo_ws/install.previous.<UTC timestamp>`; and
- restarts a previously active `savo_core.service` or `savo_edge.service`
  unless `--no-restart` is supplied.

`--allow-dirty` cannot be combined with `--pull`. Despite the current error
message calling it a "build-only update", the implementation still proceeds
to service stop, staged build/test, install activation, and optional restart.
Treat it as a development escape hatch, not a release-qualification path.

Review [production startup](production_startup.md),
[systemd services](systemd_services.md), and
[recovery operations](recovery_operations.md) before updating a robot.

## Evidence record

Record the following for every target build:

```text
Date and timezone:
Host and role:
Operating system and architecture:
ROS distribution:
Repository commit:
Dependency-install command:
Build command:
Packages selected from the role array:
Build result:
Test-result summary:
Operator:
Notes or deviations:
```
