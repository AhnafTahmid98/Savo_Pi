# Observer workstation setup

## Purpose and authority

The observer is a read-only Ubuntu 24.04 / ROS 2 Jazzy engineering workstation for telemetry, RViz, and a local dashboard. Its package validation rejects mutation clients, services, actions, velocity topics, goal tools, and initial-pose tools. It must never gain motor or mission authority.

## Prerequisites and installation

Install ROS 2 Jazzy on a workstation that can route to the robot network. Obtain the authorized repository and record its revision. Then resolve only the observer manifest and build through the supported script:

```bash
cd "$HOME/Savo_Pi"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths savo_ws/src/shared/savo_observer \
  --ignore-src -r -y --rosdistro jazzy
bash deploy/observer/build_observer.sh
bash deploy/observer/validate_observer.sh
```

Required result: `savo_observer` and its dependencies build, and validation prints `Robot SAVO observer source validation: PASS`.

## Network configuration

Set the same reviewed ROS domain and compatible RMW as Core and Edge. Do not set `ROS_LOCALHOST_ONLY=1` for cross-host observation.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
env | grep -E 'ROS_DOMAIN_ID|RMW_'
ping -c 3 192.168.50.1
ping -c 3 192.168.50.2
bash deploy/observer/check_connection.sh
```

The addresses shown are repository defaults; use the rendered deployment addresses if changed. See [ROS networking](ros_domain_networking.md).

## Run observer surfaces

Full observer:

```bash
bash deploy/observer/run_observer.sh
```

RViz only:

```bash
bash deploy/observer/run_rviz.sh
```

Dashboard profile:

```bash
bash deploy/observer/run_dashboard.sh
```

The dashboard defaults to `127.0.0.1:8765`, keeping it local to the workstation. The runner disables camera preview and point clouds by default. Do not expose it broadly without a separate reviewed access-control design.

## Verification and expected result

Confirm robot telemetry and TF appear, no mutation tool is present, and `ros2 node list` sees expected Core/Edge nodes. A working browser or RViz display proves observation only; it does not establish sensor correctness or robot readiness.

## Failure handling and evidence

For no connection, check route/ping, domain, RMW installation, `ROS_LOCALHOST_ONLY`, clock, firewall, and workspace sourcing before changing DDS behavior. Retain the revision, build/validator output, environment names, route/ping result, and connection-check output. Continue with the [commissioning checklist](commissioning_checklist.md).
