# Sign Conventions — savo_localization

This document defines the sign and frame conventions used in `savo_localization` for Robot Savo.

Keeping these conventions consistent is critical for:

- wheel odometry
- IMU integration
- EKF fusion (`robot_localization`)
- downstream navigation behavior

---

## Coordinate Convention (ROS Standard Baseline)

We follow standard ROS mobile robot conventions:

- **+X** = forward
- **+Y** = left
- **+Z** = upward

### Angular convention

- **+yaw / +angular.z** = counter-clockwise (CCW, left turn)
- **-yaw / -angular.z** = clockwise (CW, right turn)

---

## Canonical Frame Names (Phase 1 Baseline)

Use these names consistently:

- **`odom`** — local odometry/world frame
- **`base_link`** — robot base frame
- **`imu_link`** — IMU sensor frame

These frame names should match across:

- IMU node parameters
- Wheel odom node parameters
- EKF config (`ekf_odom.yaml`)
- TF consumers
- Dashboard/debug tools

---

## Required Validation Behavior

### Wheel odometry (`/wheel/odom`)

When observing `twist.twist`:

- Forward motion → `linear.x > 0`
- Reverse motion → `linear.x < 0`
- CCW rotate (left turn) → `angular.z > 0`
- CW rotate (right turn) → `angular.z < 0`

> If signs are reversed, fix via parameters first (invert flags / sign multipliers), not code rewrite.

---

### IMU (`/imu/data`)

For yaw-rate behavior (`angular_velocity.z`):

- CCW rotation (left turn) should produce **positive** `angular_velocity.z`
- CW rotation (right turn) should produce **negative** `angular_velocity.z`

If not, verify:

- IMU axis alignment
- Sensor mounting orientation
- Frame assignment (`imu_link`)
- Software axis remap/sign correction (if implemented)

---

## TF Ownership Rule (Important)

### EKF baseline rule

- `robot_localization/ekf_node` publishes `odom -> base_link`
- `wheel_odom_node` must **not** publish TF when EKF is active

### Why

Duplicate TF publishers for the same transform cause conflicts, jumps, and unstable fused localization.

---

## Practical Sign Test Procedure (Quick)

### 1) Forward test

Drive/push the robot forward slowly.

- Expect `/wheel/odom.twist.twist.linear.x > 0`

### 2) Reverse test

Drive/push the robot backward slowly.

- Expect `/wheel/odom.twist.twist.linear.x < 0`

### 3) Rotate CCW test (left turn)

Rotate the robot counter-clockwise.

- Expect `/wheel/odom.twist.twist.angular.z > 0`
- Expect `/imu/data.angular_velocity.z > 0`

### 4) Rotate CW test (right turn)

Rotate the robot clockwise.

- Expect `/wheel/odom.twist.twist.angular.z < 0`
- Expect `/imu/data.angular_velocity.z < 0`

---

## If Signs Are Wrong — Fix Priority

Use this order:

1. **Parameter fix** (preferred)
   - Encoder invert flags
   - Wheel sign multipliers
   - Axis sign parameters (if available)

2. **Frame/config fix**
   - Wrong frame assignment
   - Incorrect mounting assumption

3. **Code fix** (only if parameterization is insufficient)

---

## Scale Validation Notes (Wheel Odom)

After signs are correct, validate scale:

- Move approximately **1 meter** straight
- Compare odom-reported displacement vs actual distance
- Tune parameters if needed:
  - wheel diameter/radius
  - encoder CPR / decoding / gear ratio
  - track width

Do **not** tune EKF parameters before raw wheel odom signs/scales are reasonable.

---

## Notes for Future 4-Encoder Mecanum Upgrade (Phase 3)

When upgrading to full mecanum odometry:

- Keep the same global sign conventions:
  - forward `+x`
  - left lateral `+y`
  - CCW yaw `+z`
- Re-test all signs and scales before EKF tuning
- Keep the EKF pipeline unchanged initially (replace odom model first)

---

## Change Log (Optional)

Use this section to track sign-related changes during bringup.

- 2026-02-22: Initial sign conventions documented for Phase 1 baseline
