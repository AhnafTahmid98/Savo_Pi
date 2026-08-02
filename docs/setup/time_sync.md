# Time synchronization

Core is the preferred Chrony source for the direct link. It follows Internet NTP
when available and serves the link; orphan stratum 10 keeps both Pis mutually
coherent during Internet loss. Edge prefers core and falls back to Internet NTP.

Render `chrony_core.conf` and `chrony_edge.conf` with the network renderer, review
the subnet/IP, install them as target-specific Chrony configuration, restart
Chrony, and inspect `chronyc sources -v` and `chronyc tracking`. Do not proceed to
sensor fusion if offset is unstable or timestamps move backward.
