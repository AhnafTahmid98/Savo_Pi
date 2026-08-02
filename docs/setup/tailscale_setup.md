# Optional Tailscale access

Tailscale is optional and is **not** required for Robot SAVO core↔edge DDS or normal observer operation. The production baseline uses the direct core↔edge Ethernet link and a trusted local LAN for observers.

## Safety boundary

Do not expose ROS 2 DDS, the SavoMind speech socket, the bridge command socket, or the unauthenticated observer dashboard directly to the public internet. A Tailscale interface may be used for SSH administration after firewall review, but it must not become an implicit command or DDS transport path.

## Suggested installation

Install Tailscale using the vendor-supported Ubuntu 24.04 procedure on the computer that requires remote administration. Authenticate it to the intended private tailnet and verify:

```bash
ip -brief address show tailscale0
tailscale status
```

## Firewall policy

Allow only explicitly required administrative traffic, normally SSH from approved tailnet peers. Keep these local-only unless a separate security review authorizes them:

- `/run/savo_bridge/command.sock`
- `/run/savomind/speech.sock`
- observer dashboard port `8765`
- DDS discovery and data ports

## ROS environment

Do not change `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, DDS interface selection, or the core↔edge Netplan configuration merely to enable Tailscale. If routed DDS is later required, validate it separately with multicast/firewall tests and keep motor startup in `STOP`.

## Removal

Disabling or removing Tailscale must not affect core↔edge Ethernet operation. Verify the direct link and safe-idle bringup after any network change.
