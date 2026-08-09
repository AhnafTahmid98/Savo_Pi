# Development environment

## Purpose and target

This guide prepares an Ubuntu 24.04 / ROS 2 Jazzy engineering host to inspect, build, and test Robot Savo. Raspberry Pi role commissioning still has to run on the intended ARM64 target.

## Prerequisites

Install ROS 2 Jazzy using the official ROS Ubuntu procedure, including the ROS APT repository. The repository scripts expect `/opt/ros/jazzy/setup.bash`. A qualified engineer must also have Git access and enough free disk space; the role installer requires at least 8 GiB before dependency/build work.

Obtain the authorized Git remote from the project maintainer. No public clone URL or required branch is established by this repository.

```bash
git clone <authorized-repository-url> "$HOME/Savo_Pi"
cd "$HOME/Savo_Pi"
git status --short
git rev-parse HEAD
git diff --check
```

`<authorized-repository-url>` is intentionally host/project specific; never paste private key material or embedded credentials into the command.

## Dependency installation

On a target Pi, use only its role:

```bash
bash deploy/common/install_role_deps.sh --role core
bash deploy/common/install_role_deps.sh --role edge
```

The installer verifies Ubuntu 24.04, architecture, disk space, the role package array, ROS setup, APT dependencies, role-scoped `rosdep`, and a role build. It logs the run. Do not run both role commands as target evidence on one robot host.

On an Ubuntu 24.04 development PC only, the explicit override permits role source work:

```bash
SAVO_ALLOW_PC_INSTALL=true \
  bash deploy/common/install_role_deps.sh --role core --dry-run
```

Remove `--dry-run` only after reviewing the proposed host changes. The override does not make a PC a production Core or Edge.

For the observer, resolve its manifest dependencies and build using the repository entry point:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths savo_ws/src/shared/savo_observer \
  --ignore-src -r -y --rosdistro jazzy
bash deploy/observer/build_observer.sh
bash deploy/observer/validate_observer.sh
```

## Build and test conventions

```bash
bash deploy/core/build_core.sh --clean --test
bash deploy/edge/build_edge.sh --clean --test
bash deploy/observer/build_observer.sh
```

Core has exactly 14 required packages and Edge exactly 10, as defined in their `env_core.sh` and `env_edge.sh`. Do not bypass those arrays or use an allow-missing option as release evidence.

Source a completed workspace with:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
```

Run source validators from the repository root:

```bash
bash deploy/common/validate_full_bringup.sh
bash deploy/observer/validate_observer.sh
bash deploy/common/validate_pre_real_test_readiness.sh
```

## Repository rules

- Do not commit `savo_ws/build`, `savo_ws/install`, `savo_ws/log`, runtime databases, maps, bags, model files, or secrets.
- Do not use another host's install tree as target build evidence.
- Keep production authority paths in C++; Python fallbacks and diagnostics are not substitutes.
- Preserve role package arrays, geometry lock, supervisor gates, and `STOP` defaults.
- Use `git diff --check` and retain validator/build/test output with the revision.

## Failure handling, security, and next step

Resolve OS, APT, `rosdep`, compiler, and missing-package failures rather than manually adding unrelated workspace packages. Never place tokens in Git remotes, shell examples, logs, or tracked configuration. Continue with [Core setup](savo_core_setup.md), [Edge setup](savo_edge_setup.md), or [observer setup](observer_pc_setup.md).
