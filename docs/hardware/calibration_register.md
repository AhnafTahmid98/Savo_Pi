# Calibration Register

`MEASURED` records an owner-supplied physical value. `DERIVED` is calculated
from measured/configured geometry. `PENDING` requires physical runtime evidence.

| Calibration item | Current value/state | Status | Remaining evidence |
| --- | --- | --- | --- |
| Wheel diameter/radius | `0.065 / 0.0325 m` | MEASURED/configured | Loaded rolling-distance trials |
| Wheelbase / track / mecanum k | `0.160 / 0.216 / 0.188 m` | MEASURED/DERIVED | Odometry scale and yaw trials |
| Wheel centers | X `+/-0.080`, Y `+/-0.108 m` | MEASURED | Review after bracket/wheel changes |
| Encoder CPR/decoding/gear | `20 / x4 / 1.0` | CONFIGURED | Shaft/wheel revolution counts |
| Encoder and motor polarity | source values | PENDING | Wheels-raised sign test |
| Base frame height | `0.0325 m` axle plane | CONVENTION/DERIVED | Preserve frame contract |
| Plate XYZ | `0.2796 x 0.2100 x 0.0040 m`; Z `.014/.080/.200` | MEASURED, provisional datum | Resolve surface versus center-plane datum (`2 mm` ambiguity) |
| BNO055 position | ground `[0,-.0465,.015]` | MEASURED | Establish board +X/+Y, bias/noise and magnetic survey |
| LiDAR position | ground `[0,0,.330]` | MEASURED | Verify scan-zero yaw with front obstacle |
| D435 mount | ground `[.130,0,.225]`, RPY zero | MEASURED | Validate internal stream extrinsics/depth/VO |
| Side ToF mounts | ground `[0,+/-.106,.025]` | MEASURED | Live left/right beam check |
| Front ultrasonic | ground `[.137,0,.056]` | MEASURED | Live beam/FOV check |
| Head translations | pan/tilt/lens ground positions measured | MEASURED | Validate pan_sign/tilt_sign and runtime TF |
| Head neutral | pan `72 deg`, tilt `55 deg` | CONFIGURED | Mechanical center/clearance |
| Wheel width/mass/inertials | source provisional values | PENDING | Physical survey/weighing |
| Display/ReSpeaker/mass geometry | legacy profile values | PENDING | Physical survey |

The geometry profile remains `provisional`. A dated `VALIDATED` entry must retain
robot revision, commit, operator, instrument, environment/load, raw artifact,
calculation, tolerance, reviewer, and configuration change reference.
