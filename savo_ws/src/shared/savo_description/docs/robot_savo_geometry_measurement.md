# Robot Savo geometry measurement record

Profile revision 3 integrates owner-supplied physical measurements dated
2026-08-20. Coordinates use +X forward, +Y left, +Z up.

## Measured

- Four plates, all `0.0040 m` thick. The base, first, and second plates are
  `0.2796 x 0.2100 m`; the third LiDAR-only plate is `0.1420 x 0.1440 m`.
- `Base_Layer.STL` includes the DC-motor mounts, producing a `0.0280 m` total
  mesh envelope. The physical base plate itself remains `0.0040 m` thick.
- Authoritative reported plate ground Z: base `0.015`, first `0.080`, second
  `0.196`, and third `0.280 m`.
- Center-plane separations derived from those absolute heights are `0.065 m`
  base-to-first, `0.116 m` first-to-second, and `0.084 m` second-to-third.
- Wheel centers: front X `+0.080`, rear X `-0.080`, left Y `+0.108`, right Y `-0.108 m`.
- Wheel diameter `0.065 m`; wheelbase `0.160 m`; track `0.216 m`.
- IMU ground XYZ `[0,-0.0465,0.015] m`; BNO055 +Z points up.
- LiDAR ground XYZ `[0,0,0.330] m`.
- D435 ground XYZ `[0.130,0,0.225] m`, mount RPY `[0,0,0]`.
- Left/right ToF ground XYZ `[0,+/-0.106,0.025] m`, facing +Y/-Y.
- Front ultrasonic ground XYZ `[0.137,0,0.056] m`, facing +X.
- Pan axis `[0.115,0,0.244]`, tilt axis `[0.115,0,0.290]`, and Pi-camera lens `[0.140,0,0.280] m` from ground at neutral.

## Derived frame convention

`base_footprint` is the robot center projected onto the ground. `base_link` is
the axle plane at `+0.0325 m`, derived from wheel radius. It is not a direct
height measurement. All fixed-mount `base_link` Z values subtract `0.0325 m`
from their ground Z.

## Provisional modeling decisions

The supplied plate Z datum is not proven to be a surface or center plane.
Because URDF box origins are center planes, the model provisionally treats the
reported Z values as centers and records a `0.002 m` ambiguity. D435 internal
color/depth frame translations remain zero placeholders, not calibrated stream
extrinsics. Legacy display/ReSpeaker positions, wheel width, masses, and
inertials also remain provisional.

## Layer equipment layout

- Base: DC motors, mecanum wheels, ToFs, front ultrasonic, IMU, and multiplexer.
- First: display, speakers, Core Pi, and breadboard.
- Second: RealSense D435, ReSpeaker microphone array, Edge Pi, and pan-tilt
  camera; the pan-tilt assembly is above the RealSense.
- Third: RPLIDAR A1 only.

## Required runtime verification before lock

1. Establish BNO055 +X/+Y orientation relative to the robot.
2. Verify LiDAR scan-zero yaw in RViz with an obstacle directly ahead.
3. Obtain authoritative D435 internal stream extrinsics or safely redesign TF ownership.
4. Complete the Pi runtime TF validation for the measured head chain before
   closing its calibration blocker.
5. Resolve each reported plate Z datum as surface or center plane.
6. Measure wheel width and physical mass/inertial values.
7. Survey the full collision envelope and remaining display/ReSpeaker geometry.

Do not change `measurement_state` to `locked` until these blockers have dated,
reviewed evidence.
