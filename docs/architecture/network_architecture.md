# Network architecture

Core and edge communicate over dedicated Ethernet using ROS 2 DDS. Core owns
motion/safety/localization/mapping/navigation; edge owns D435/VO/speech/UI,
SavoMind, and the native bridge. Matching `ROS_DOMAIN_ID` and
`RMW_IMPLEMENTATION` are mandatory. Wi-Fi is management/Internet failover, not a
replacement safety channel.

The SavoMind container reaches ROS only through the bounded, credential-checked
Unix socket owned by `savo_bridge`. Observer computers receive read-only DDS and
dashboard data. Firewalls should allow DDS only on trusted interfaces. Core is
the preferred time source over Ethernet; edge fails over to Internet Chrony.
