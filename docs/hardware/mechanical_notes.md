# Mechanical Notes

## Measured geometry integrated on 2026-08-11

All three plates measure `0.2796 x 0.2100 x 0.0040 m`. The owner reported
ground-referenced Z values of `0.014`, `0.080`, and `0.200 m` for the base,
first, and second plates. The measurement datum is not yet proven to be a top
surface or a center plane. URDF box origins provisionally use the reported
values as center-plane heights, which leaves an explicit `0.002 m` vertical
ambiguity for each plate.

`base_footprint` is the robot XY center projected onto the ground. `base_link`
is intentionally the axle-plane frame at `+0.0325 m`, derived from the
configured/measured `0.065 m` wheel diameter. This is a frame convention, not a
direct measurement of `base_link` height.

Measured wheel centers are FL `[+0.080,+0.108]`, FR
`[+0.080,-0.108]`, RL `[-0.080,+0.108]`, and RR
`[-0.080,-0.108] m`. The full wheelbase is `0.160 m`, full track is
`0.216 m`, and mecanum `k=(wheelbase+track)/2` is `0.188 m`.

Wheel width (`0.030 m`), masses, inertials, display/ReSpeaker geometry, and the
complete rigid collision envelope remain provisional. The production Nav2
footprint therefore remains the conservative `0.330 x 0.240 m` rectangle with
separate `0.020 m` padding; it is intentionally not reduced to the plate-only
envelope.

## Serviceability and risk controls

- Preserve LiDAR sweep, camera/depth fields of view, ToF/ultrasonic beams, microphone aperture, airflow, and Wi-Fi antennas.
- Keep wiring away from mecanum rollers, servo travel, sharp edges, hot components, and removable plate fasteners.
- Provide strain relief at USB, CSI, display, motor, encoder, and battery connections.
- Keep emergency power control reachable and label moving/crush zones.

The remaining physical survey includes maximum rigid envelope, wheel tread
width, mass/centre of gravity, plate datum, deflection, head sweep/cable bend,
sensor occlusion, thermal clearance, fastener retention, and tip stability.
