# Environment and secrets

## Purpose

Robot Savo separates reviewable non-secret runtime settings from host-local credentials. This guide covers the role scripts and optional systemd environment file; provider secrets remain outside this repository.

## Non-secret environment

`deploy/core/env_core.sh`, `deploy/edge/env_edge.sh`, and `deploy/common/env_common.sh` define role defaults. Important configurable values include:

| Value | Current default or rule |
| --- | --- |
| `SAVO_ROOT` | `$HOME/Savo_Pi` unless explicitly set |
| `SAVO_WS` | `$SAVO_ROOT/savo_ws` |
| `ROS_DISTRO` | `jazzy` |
| `ROS_DOMAIN_ID` | `0`, but all hosts must use the same reviewed value |
| `ROS_LOCALHOST_ONLY` | `0` |
| `SAVO_ROLE` | Set by the selected Core or Edge role |
| `SAVO_ROBOT_MODE` | `safe_idle` |
| `SAVO_BRINGUP_PROFILE` | `lidar_only` |
| `SAVO_CONTROL_STARTUP_MODE` | `STOP` |

The shared systemd example also sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; the shell role environment does not impose an RMW. Use a compatible installed RMW on every participating host.

## Protected systemd environment

From the repository root:

```bash
sudo install -d -m 0750 /etc/robot-savo
sudo install -m 0640 deploy/systemd/robot-savo.env.example \
  /etc/robot-savo/robot-savo.env
sudoedit /etc/robot-savo/robot-savo.env
```

Set the absolute repository/workspace paths, correct `SAVO_ROLE`, common ROS domain/RMW, dedicated network interface names, and only reviewed feature flags. Keep `safe_idle`, `lidar_only`, `STOP`, locked geometry, provisional-geometry rejection, and unvalidated D435 obstacle integration unchanged during setup.

The role-specific units use `EnvironmentFile=-/etc/robot-savo/robot-savo.env`; a missing file is allowed, but commissioning must record the effective environment.

## Secrets boundary

No current Core or Edge role requires a provider API key in tracked ROS YAML. Model-provider credentials and remote-service tokens, if used by SavoMind, belong in the authoritative SavoMind deployment's protected secret mechanism.

Never put secrets in:

- Git-tracked Markdown, YAML, launch files, or unit files;
- a Git remote URL;
- command-line examples that remain in shell history;
- ROS parameters, topics, snapshots, or diagnostic logs;
- world-readable environment files.

Restrict any host-local secret file to its service identity, avoid printing values during diagnostics, and rotate a credential if it appears in logs or Git history. Placeholder names such as `PROVIDER_API_KEY` describe a class of secret, not an actual value.

## Verification and evidence

Inspect names without dumping secret values:

```bash
systemctl show savo_core.service -p EnvironmentFiles
systemctl show savo_edge.service -p EnvironmentFiles
env | grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|SAVO_ROLE)='
stat -c '%A %U:%G %n' /etc/robot-savo /etc/robot-savo/robot-savo.env
git status --short
```

Use the applicable service name on each host. Retain the configuration revision and a redacted setting inventory. Continue with [ROS domain networking](ros_domain_networking.md) and [systemd services](../deployment/systemd_services.md).
