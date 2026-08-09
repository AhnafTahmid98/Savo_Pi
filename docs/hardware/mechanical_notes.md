# Mechanical Notes

## Current configured model

The provisional `robot_savo_core_v1` model configures a `0.330 x 0.240 x 0.060 m` base, `base_link` at z `0.065 m`, three `0.004 m` decks at `0.070 m` spacing, `0.0325 m` wheel radius, and `0.030 m` wheel width. Configured mass values are visualization/inertial inputs and have not been confirmed by a weighed assembly.

Wheel centers are modeled at x `+/-0.115 m`, y `+/-0.100 m`, z `-0.030 m`. This implies a `0.230 m` wheelbase and `0.200 m` track, inconsistent with the `0.165/0.165 m` base/localization kinematic configuration. Do not lock either representation until measured from the assembled wheel contact geometry.

## Serviceability and risk controls

- Preserve LiDAR sweep, camera/depth fields of view, ToF/ultrasonic beams, microphone aperture, airflow, and Wi-Fi antennas.
- Keep wiring away from mecanum rollers, servo travel, sharp edges, hot components, and removable deck fasteners.
- Provide strain relief at USB, CSI, display, motor, encoder, and battery connections.
- Keep the emergency power control reachable and label moving/crush zones.
- Document fastener type/length, standoff height, bracket revision, thread locking, and torque only after physical inspection; no torque values are currently authoritative.

Commissioning measurements include overall envelope, ground clearance, wheel contact rectangle, mass/centre of gravity, deck deflection, pan/tilt sweep and cable bend, sensor occlusion, fan/thermal clearance, fastener retention, and tip stability. Photos should show datum axes, each layer, cable routing, and serial labels without exposing secrets.
