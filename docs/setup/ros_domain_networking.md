# ROS domain networking

## Purpose

Core, Edge, and observer must share a reachable IP network, a compatible DDS implementation, and one reviewed ROS domain. ROS discovery is required for distributed safe idle but is not proof of readiness.

## Current contract

`deploy/common/env_common.sh` defaults `ROS_DOMAIN_ID=0`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`; `ROS_LOCALHOST_ONLY` is intentionally unset. The domain remains configurable, but every participating host must use the reviewed matching FastDDS/domain/discovery contract.

Do not hard-code domain `0` without reviewing nearby robots and networks. Record the chosen value, use it on all three hosts, and avoid overlapping fleets on the same Layer-2 network.

## Persistent configuration

For systemd roles, copy and edit the protected environment as described in [environment and secrets](environment_and_secrets.md). For an interactive commissioning shell, set the same reviewed values used by its role service:

```bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
```

The value `0` above matches the repository default; replace it consistently if the deployment record selects another domain. Confirm the selected RMW package is installed rather than changing implementations to mask routing problems.

## Verification

On every host:

```bash
env | grep -E 'ROS_DOMAIN_ID|ROS_AUTOMATIC_DISCOVERY_RANGE|RMW_'
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

For one-host-only discovery, check that `ROS_LOCALHOST_ONLY` is unset, then check discovery range, domain, FastDDS installation, IP route, firewall/multicast policy, service environment versus shell environment, and time synchronization. `deploy/common/diagnose_fastdds_shm.py` reports SHM candidates and active ROS processes without changing them. Candidate files do not prove causality; never clean SHM during a live robot session and never use forced cleanup.

Retain environment names (not secrets), IP/routes, ping results, node lists, hostnames, and timestamps. Related guidance: [network and time](network_and_time_setup.md) and [network architecture](../architecture/network_architecture.md).
