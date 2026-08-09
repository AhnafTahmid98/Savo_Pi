# Network Architecture

## Topology and defaults

```text
Internet/admin Wi-Fi                 Internet/admin Wi-Fi
       |                                    |
   savo-core 192.168.50.1/24 --- 192.168.50.2/24 savo-edge
                         dedicated Ethernet
```

`deploy/network/network.env.example` defaults both Ethernet interfaces to `eth0`, Wi-Fi to `wlan0`, and the direct subnet to `192.168.50.0/24`. Interface names and credentials must be verified per host before rendering. Netplan disables DHCP, IPv6 DHCP, and link-local addressing on the robot Ethernet; Wi-Fi uses DHCP.

## ROS and time

Both hosts require the same `ROS_DOMAIN_ID` (default `0`), `ROS_LOCALHOST_ONLY=0`, ROS 2 Jazzy installation, and compatible RMW implementation. The systemd environment example selects `rmw_cyclonedds_cpp`. A production deployment must verify DDS discovery is bound only to trusted interfaces; matching variables alone do not prove firewall or multicast behavior.

Chrony makes Core the preferred isolated-link source. Core can synchronize from Internet pools and advertises time to the robot subnet; Edge prefers Core and can use Internet pools as fallback. Clock health is a hard prerequisite for timestamp freshness, TF, sensor fusion, and distributed timeout decisions.

## Security boundary

The Ethernet is a trusted robot segment. Wi-Fi/Tailscale may provide SSH administration after firewall review, but must not implicitly route or publish DDS. `/run/savo_bridge/command.sock` and `/run/savomind/speech.sock` are Unix sockets local to Edge and are never TCP services. The observer is read-only and must not be exposed unauthenticated to the public Internet.

## Failure behavior and validation

Loss of the link makes remote inputs stale. Bridge actions must fail closed, Core readiness applies its configured required dependencies, and local Core command/safety watchdogs remain authoritative. Network restoration does not itself re-arm the robot.

Validate rendered Netplan without applying it first, then verify interface addresses/routes, `ping`, `ros2` discovery, Chrony tracking/sources, firewall rules, middleware identity, throughput for selected RGB-D flows, link loss, and reconnect while Core remains in `STOP`. Repository render/health validators are source-level evidence; the actual two-Pi network remains target-dependent.
