# Measurement Checklist

Use a revision-controlled worksheet and calibrated tools. Record value, units, uncertainty, method, tool ID/calibration date, operator, date, robot hardware revision, source commit, and photo/evidence path. Never replace a configured value without review.

## Geometry and mechanics

- [ ] Define and photograph `base_footprint` and `base_link` datums and ROS axes.
- [ ] Measure base envelope, deck sizes/thickness/spacing, ground clearance, bumper envelope, and Nav2 footprint.
- [ ] Measure wheel effective rolling radius, width, x/y contact centres, wheelbase, track, roller orientation, and polarity.
- [ ] Weigh complete robot and major removable assemblies; estimate centre of gravity and tip margin.
- [ ] Measure every fixed sensor origin/orientation and head kinematic link/extrinsic.
- [ ] Verify head sweep, cable bend radius, sensor occlusion, and moving-part clearance.

## Electrical and power

- [ ] Trace power tree, chemistry/capacity, nominal/min/max voltage, converter, switch, fuse, connector, wire gauge, and ground return.
- [ ] Verify GPIO/I2C/USB/CSI/display/audio cable endpoints, pin 1, voltage levels, pull-ups, shielding, and strain relief.
- [ ] Calibrate ADS7830 voltage conversion and UPS telemetry against a traceable meter.
- [ ] Measure idle/typical/start/stall current, voltage sag, thermal rise, runtime, and brownout recovery.

## Sensor/actuator calibration

- [ ] Encoder counts/revolution, direction, missed counts, straight-line scale, lateral scale, and yaw scale.
- [ ] BNO055 axis, calibration state, magnetic interference, bias/noise, and covariance.
- [ ] LiDAR rate/range/angle/frame, ToF aim/range/cross-talk, ultrasonic level/beam/stale behavior.
- [ ] D435 serial/USB mode/rates/alignment/depth scale/extrinsic and VO quality.
- [ ] Servo centre/min/max, pulse limits, mechanical stops, backlash, TF zero/sign.
- [ ] Motor channel/polarity, wheel-raised watchdog, duty response, and stop latency.

## Closure

- [ ] Resolve wheel-model versus kinematic geometry mismatch.
- [ ] Validate shared PCA9685 base/head startup and concurrent access.
- [ ] Update and review geometry/calibration/configuration; regenerate footprint/URDF digest.
- [ ] Mark the profile locked only after independent review and archive evidence.
