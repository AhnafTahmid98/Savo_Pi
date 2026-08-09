# Network and time setup

## Purpose and target

This procedure configures the dedicated Core–Edge link, optional upstream Wi-Fi, and Chrony. The repository defaults are Core `192.168.50.1/24`, Edge `192.168.50.2/24`, and subnet `192.168.50.0/24`. Interface names and credentials are host-specific and must be discovered, not assumed.

Network changes can disconnect SSH. Use a local console or tested recovery path before installing or applying Netplan.

## Identify interfaces

On each Pi:

```bash
ip -br link
ip -br address
ip route
```

Record the actual dedicated Ethernet and upstream interfaces. Do not copy the `eth0`/`wlan0` example values unless the target confirms them.

## Render configuration

Create a protected host-local input from the tracked example and edit its values:

```bash
install -d -m 0700 "$HOME/robot-savo-network"
install -m 0600 deploy/network/network.env.example \
  "$HOME/robot-savo-network/network.env"
chmod 0600 "$HOME/robot-savo-network/network.env"
```

Set both hosts' actual interface names, reviewed addresses/prefix/subnet, Wi-Fi SSID/password, and the deployment ROS domain. The renderer refuses `CHANGE_ME` credentials.

Render each role without applying it:

```bash
bash deploy/network/render_network_config.sh \
  --role core \
  --env "$HOME/robot-savo-network/network.env" \
  --output-dir "$HOME/robot-savo-network/rendered-core"
bash deploy/network/render_network_config.sh \
  --role edge \
  --env "$HOME/robot-savo-network/network.env" \
  --output-dir "$HOME/robot-savo-network/rendered-edge"
```

The script renders Netplan and Chrony files and may schema-check Netplan. It prints `Rendered configuration only; nothing was applied.` Review the output locally; it contains the Wi-Fi password and must remain protected.

## Install and apply carefully

On Core, install its two reviewed files; use the equivalent `edge` names on Edge:

```bash
sudo install -m 0600 \
  "$HOME/robot-savo-network/rendered-core/50-robot-savo-core.yaml" \
  /etc/netplan/50-robot-savo-core.yaml
sudo install -d -m 0755 /etc/chrony/conf.d
sudo install -m 0644 \
  "$HOME/robot-savo-network/rendered-core/chrony-core.conf" \
  /etc/chrony/conf.d/robot-savo.conf
sudo netplan generate
sudo netplan try
```

`netplan try` is interactive and can roll back. Confirm only from local console after addresses, Wi-Fi, routes, and recovery access work. Then restart Chrony:

```bash
sudo systemctl restart chrony
systemctl status chrony --no-pager
```

Core uses Internet pools when reachable, serves the dedicated subnet, and provides orphan local stratum 10. Edge prefers Core and uses Internet pools as fallback.

## Verification

From Core and Edge, substitute the configured peer address if defaults changed:

```bash
ip -br address
ip route
ping -c 3 192.168.50.2
ping -c 3 192.168.50.1
chronyc tracking
chronyc sources -v
```

Verify the appropriate peer ping on each host, no unintended default route on the dedicated link, Edge selecting Core when available, and acceptable clock agreement before distributed ROS testing. Time agreement is required for TF, sensor stamps, VO, localization, speech correlation, and comparable logs.

After setting the common ROS environment, run `ros2 node list` on both hosts and the observer. See [ROS domain networking](ros_domain_networking.md).

## Failure handling and security

- Missing interface: compare `ip -br link` with the rendered names; do not guess or rename blindly.
- Wrong address/route: use local console, allow `netplan try` to roll back, then correct the host-local input.
- No discovery: verify ping first, then domain/RMW/firewall and clock.
- Large clock offset: inspect `chronyc tracking` and `sources -v`; do not proceed to timestamp-sensitive validation.
- Netplan schema blocked by D-Bus in a container: render is not target installation evidence; validate on the actual host.

Keep the host-local input and rendered Netplan protected because they contain Wi-Fi credentials. Retain redacted input values, installed file hashes, interface/address/route output, and Chrony evidence.
