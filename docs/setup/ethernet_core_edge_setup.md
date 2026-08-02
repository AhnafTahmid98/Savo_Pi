# Core-edge Ethernet setup

Use a dedicated Ethernet link with defaults core `192.168.50.1/24` and edge
`192.168.50.2/24`; validate the actual interface names with `ip -br link` first.
Copy `deploy/network/network.env.example` outside the repository, enter real
interfaces and Wi-Fi credentials, then render without applying:

```bash
deploy/network/render_network_config.sh --role core --env /secure/network.env --output-dir /tmp/savo-core-network
deploy/network/render_network_config.sh --role edge --env /secure/network.env --output-dir /tmp/savo-edge-network
```

Review the files, run `netplan generate` on the target Pi, and only then let an
operator install/apply them locally. Never apply network changes over an
unrecoverable remote-only session. Both hosts must use the same ROS domain and
RMW implementation. Do not route motor/control traffic through the observer.
