# Robot Savo geometry measurement record

Profile revision 5 integrates owner-supplied physical measurements and the
physical D435 factory calibration through 2026-08-29. Coordinates use +X
forward, +Y left, +Z up.

## Measured

- Four plates, all `0.0040 m` thick. The base, first, and second plates are
  `0.2796 x 0.2100 m`; the third LiDAR-only plate is `0.1420 x 0.1440 m`.
- `Base_Layer.STL` includes the DC-motor mounts, producing a `0.0280 m` total
  mesh envelope. The physical base plate itself remains `0.0040 m` thick.
- Authoritative plate bottom-surface ground Z: base `0.015`, first `0.080`,
  second `0.196`, and third `0.280 m`. With `0.004 m` thickness, their center
  ground Z values are `0.017`, `0.082`, `0.198`, and `0.282 m`.
- Center-plane separations derived from those absolute heights are `0.065 m`
  base-to-first, `0.116 m` first-to-second, and `0.084 m` second-to-third.
- Wheel centers: front X `+0.080`, rear X `-0.080`, left Y `+0.108`, right Y `-0.108 m`.
- Wheel diameter `0.065 m`; wheelbase `0.160 m`; track `0.216 m`.
- IMU ground XYZ `[0,-0.0465,0.015] m`; later hardware calibration closed its
  board orientation at mount RPY `[0,0,0]`.
- LiDAR ground XYZ `[0,0,0.330] m`; the final flat-door plane-fit calibration
  sets the `base_link -> laser_frame` scan-zero yaw to `-3.089891 rad`
  (`-177.038 deg`). This supersedes the earlier `-172.188 deg` cluster-center
  estimate; three independent 20-scan batches had a `0.267 deg` repeatability
  spread and `0.0095 m` maximum plane-fit RMS.
- D435 ground XYZ `[0.130,0,0.225] m`, mount RPY `[0,0,0]`. Factory
  depth/color extrinsics came from physical serial `801212070967`, firmware
  `5.16.0.1`.
- Left/right ToF ground XYZ `[0,+/-0.106,0.025] m`, facing +Y/-Y.
- Front ultrasonic ground XYZ `[0.137,0,0.056] m`, facing +X.
- Pan axis `[0.115,0,0.244]`, tilt axis `[0.115,0,0.290]`, and Pi-camera lens `[0.140,0,0.280] m` from ground at neutral.
- Complete assembled fixed-body XY survey, including wheels and permanently
  mounted fixed hardware: `0.280 x 0.280 m`.

## Derived frame convention

`base_footprint` is the robot center projected onto the ground. `base_link` is
the axle plane at `+0.0325 m`, derived from wheel radius. It is not a direct
height measurement. All fixed-mount `base_link` Z values subtract `0.0325 m`
from their ground Z.

## Production planning envelope

The authoritative raw production footprint is centered on `base_footprint`
with extents `X/Y +/-0.145 m`, giving `0.290 x 0.290 m`. Nav2 applies its
existing `0.020 m` padding once, producing approximate padded dimensions of
`0.330 x 0.330 m` and bounds `X/Y +/-0.165 m`.

## Non-blocking model-fidelity decisions

The display/ReSpeaker primitives, exact wheel width, component masses, and
inertials remain legacy model estimates rather than physical measurements.
Mass, CG, and inertia work is simulation fidelity and does not block the
production geometry lock. The current legacy display primitive reaches
`x=+0.1975 m`, but it is not a physical measurement and cannot override the
complete assembled survey. No replacement display pose is inferred.

## Layer equipment layout

- Base: DC motors, mecanum wheels, ToFs, front ultrasonic, IMU, and multiplexer.
- First: display, speakers, Core Pi, and breadboard.
- Second: RealSense D435, ReSpeaker microphone array, Edge Pi, and pan-tilt
  camera; the pan-tilt assembly is above the RealSense.
- Third: RPLIDAR A1 only.

## Lock state

All production navigation and safety-critical physical geometry evidence is
closed. `measurement_state` is `locked`; remaining detailed primitive, mass,
CG, and inertia work stays in the non-blocking model-fidelity list.
