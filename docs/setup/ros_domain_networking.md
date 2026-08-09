# ROS domain networking

## Purpose

Core, Edge, and observer must share a reachable IP network, a compatible DDS implementation, and one reviewed ROS domain. ROS discovery is required for distributed safe idle but is not proof of readiness.

## Current contract

`deploy/common/env_common.sh` defaults `ROS_DOMAIN_ID` to `0` and `ROS_LOCALHOST_ONLY` to `0`; the domain remains configurable. `deploy/systemd/robot-savo.env.example` additionally selects `rmw_cyclonedds_cpp`. The role shell files do not force an RMW, so every host must install and select a mutually compatible implementation.

Do not hard-code domain `0` without reviewing nearby robots and networks. Record the chosen value, use it on all three hosts, and avoid overlapping fleets on the same Layer-2 network.

## Persistent configuration

For systemd roles, copy and edit the protected environment as described in [environment and secrets](environment_and_secrets.md). For an interactive commissioning shell, set the same reviewed values used by its role service:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
```

The value `0` above matches the repository default; replace it consistently if the deployment record selects another domain. Confirm the selected RMW package is installed rather than changing implementations to mask routing problems.

## Verification

On every host:

```bash
env | grep -E 'ROS_DOMAIN_ID|RMW_'
ip address
ip route
ros2 node list
```

Start Core and Edge in safe idle. On Core, verify Edge nodes are visible; on Edge, verify Core nodes are visible. On the observer:

```bash
cd "$HOME/Savo_Pi"
bash deploy/observer/check_connection.sh
```

Compare `ros2 node list` from each host. Expected result is a consistent distributed graph without duplicate component owners. Discovery does not establish topic freshness, QoS compatibility, TF correctness, or motion permission; inspect those during formal validation.

## Failure handling

For one-host-only discovery, check `ROS_LOCALHOST_ONLY`, domain, RMW, IP route, firewall/multicast policy, service environment versus shell environment, and time synchronization. Restarting DDS participants may clear stale discovery, but do not disable a firewall or expose robot DDS beyond the approved network as a shortcut.

Retain environment names (not secrets), IP/routes, ping results, node lists, hostnames, and timestamps. Related guidance: [network and time](network_and_time_setup.md) and [network architecture](../architecture/network_architecture.md).
