# Two-Pi Architecture

## Role split

| Responsibility | Core (`savo-core`) | Edge (`savo-edge`) |
| --- | --- | --- |
| Motion and safety | Supervisor, control, command gate, base | No direct authority |
| Robot sensing | LiDAR, IMU, encoders, ToF, ultrasonic | RealSense D435, ReSpeaker |
| State estimation | Wheel odometry, EKF, TF authority | Optional VO producer |
| Autonomy | Mapping, map release, Nav2, locations | Typed request/observation bridge |
| Human interface | Head camera/pan-tilt | Speech and optional 800 x 480 UI |
| Power | Core UPS, base battery, aggregate | Edge UPS telemetry |

Authoritative package membership is the 14-package Core array and 10-package Edge array in `deploy/*/env_*.sh`. Folder placement alone does not imply deployment.

## Cross-host contracts

The hosts share ROS 2 Jazzy DDS over a dedicated Ethernet link, normally `192.168.50.1/24` and `192.168.50.2/24`. They require a matching `ROS_DOMAIN_ID` and middleware selection; the systemd example selects `rmw_cyclonedds_cpp`. Core is the preferred Chrony source on the isolated link.

State crossing Edge to Core includes `/vo/odom`, RealSense/VO health, optional obstacle-cloud data, Edge UPS state, and typed bridge requests. Core to Edge includes robot state, readiness, safety, map/navigation/location state, and bounded action/service results. Raw high-bandwidth RGB-D remains Edge-local unless a configured consumer requires it.

## Failure isolation

Core remains the motion authority if Edge is absent. Loss or staleness of a required Edge component blocks the applicable profile; optional components report degraded state. Edge must never compensate for a Core failure by driving `/cmd_vel_safe`, hardware, supervisor state, map release, or operator approvals.

The dedicated link is a trusted robot network, not an Internet boundary. Wi-Fi is for administration and Internet access. ROS DDS, bridge sockets, speech sockets, and the observer dashboard must not be exposed publicly.

## Startup and shutdown

Each Pi uses the same role-selecting `robot_bringup.launch.py` with its own `host_role`. Default deployment is `safe_idle`, `lidar_only`, and Core `STOP`; Edge bridge is enabled while speech, UI, and obstacle-cloud startup remain opt-in. Core state is persistent; Edge IPC sockets are runtime-only and recreated by deployment preparation.

Validation requires separate role builds, two-host discovery/time checks, independent readiness namespaces, Edge-loss tests, command freshness tests, and confirmation that Core remains stopped. Current source validators cover role membership and launch contracts; target and physical two-Pi regression remain required.

Related: [Core](savo_core_architecture.md), [Edge](savo_edge_architecture.md), [network](network_architecture.md), and [motion authority](motion_authority_model.md).

