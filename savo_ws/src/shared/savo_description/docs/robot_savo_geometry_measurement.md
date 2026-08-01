# Robot SAVO geometry measurement worksheet

The checked-in `robot_savo_core_v1` values are provisional. Keep the robot on a level surface in its normal operating configuration and record raw measurements before editing the profile. A human reviewer must fill `measured_by`, `measurement_date`, and the notes before changing `measurement_state` to `locked`.

Use metres and radians with the ROS convention: +X forward, +Y left, +Z up. Record the instrument and estimated uncertainty for every value.

1. Measure maximum chassis X, Y, and Z extents, including rigid collision-relevant parts. Do not include configurable software padding.
2. Under normal robot load, measure each wheel diameter at the loaded rolling radius and the wheel tread width. Measure every wheel centre relative to `base_link`; record front/rear X, left/right Y, and Z.
3. Establish `base_footprint` on the floor projection and measure its vertical distance to `base_link`.
4. Measure the LiDAR scan origin, BNO055 sensing origin, and BNO055 axis orientation relative to `base_link`.
5. Measure the RealSense D435 body origin and orientation. Verify the ROS optical convention separately; do not use the housing face as the optical origin without checking the device model.
6. Measure left/right ToF emitter origins and orientations, then check their Y symmetry. Measure the front ultrasonic acoustic centre.
7. Measure the pan-axis origin relative to `pantilt_mount_link`, the tilt-axis origin relative to `pantilt_pan_link`, and their positive axes.
8. Measure the Pi-camera optical centre relative to the tilt axis and verify the camera-to-optical rotation.
9. Measure the display and ReSpeaker mounting origins because they affect the fixed model and collision-envelope review.
10. Run `validate_geometry_profile.py PROFILE --require-locked`, generate the URDF, inspect the TF tree, and have a second person review the worksheet before controlled motion testing.

Never lock the profile from inferred CAD or software defaults alone.
