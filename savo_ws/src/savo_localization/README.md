# savo_localization

Robot Savo — Localization package (ROS 2 Jazzy) for IMU + wheel odometry + EKF baseline.

This package provides the localization foundation for Robot Savo using a staged approach:

- **Phase 1 (current):** IMU validation + wheel odometry validation + EKF (`robot_localization`) baseline
- **Phase 2:** Optional IMU filter comparison/tuning (e.g., Madgwick) and EKF tuning (only if needed)
- **Phase 3:** Upgrade odometry model to **4-wheel mecanum** when two additional encoders are installed

> Current baseline uses **IMU + 2 rear wheel encoders**.  
> Full mecanum odometry is intentionally postponed until 4 encoders are available.

---

## Package Purpose

This package is responsible for:

- Reading and publishing IMU data (`imu_node.py`)
- Publishing wheel odometry (primary C++ node)
- Providing a Python fallback wheel odometry node for diagnostics
- Providing a localization dashboard (terminal diagnostics)
- Running `robot_localization` EKF (`ekf_node`) to fuse IMU + wheel odom

---

## Package Layout (high level)

```text
savo_localization/
├─ README.md
├─ docs/
│  └─ sign_conventions.md
├─ config/
│  ├─ ekf_odom.yaml
│  ├─ encoders.yaml
│  ├─ frames.yaml
│  ├─ imu.yaml
│  └─ localization_common.yaml
├─ launch/
│  └─ localization_bringup.launch.py
├─ include/savo_localization/
│  └─ wheel_odom.hpp
├─ src/
│  └─ wheel_odom_node.cpp
├─ savo_localization/
│  ├─ nodes/
│  │  ├─ imu_node.py
│  │  ├─ wheel_odom_fallback_node.py
│  │  └─ localization_dashboard.py
│  ├─ sensors_api/
│  │  ├─ imu_api.py
│  │  └─ encoders_api.py
│  └─ utils/
└─ package.xml / CMakeLists.txt / setup.py
```

---

## Architecture (Current Baseline)

### Primary localization pipeline (Phase 1)

- `imu_node.py` → publishes `/imu/data`
- `wheel_odom_node.cpp` → publishes `/wheel/odom`
- `robot_localization/ekf_node` → publishes `/odometry/filtered` and TF `odom -> base_link`

### Fallback/debug pipeline

- `wheel_odom_fallback_node.py` can temporarily replace the C++ wheel odom node for diagnostics/testing.

---

## Topics

### Inputs (hardware/API side)

These are internal sources used by nodes in this package (depending on launch mode):

- BNO055 IMU (via `imu_api.py`)
- Wheel encoder signals (via `encoders_api.py`)

---

### Published Topics (main outputs)

#### IMU

- **`/imu/data`** (`sensor_msgs/msg/Imu`)
  - Published by: `imu_node.py`
  - Used by: `robot_localization/ekf_node`

#### Wheel odometry (raw)

- **`/wheel/odom`** (`nav_msgs/msg/Odometry`)
  - Published by: `wheel_odom_node` (C++) **or** `wheel_odom_fallback_node.py`
  - Used by: `robot_localization/ekf_node`

#### EKF fused odometry

- **`/odometry/filtered`** (`nav_msgs/msg/Odometry`)
  - Published by: `robot_localization/ekf_node`
  - Used by: downstream localization/navigation components

---

### Subscribed Topics (optional / diagnostics)

`localization_dashboard.py` may subscribe to:

- `/imu/data`
- `/wheel/odom`
- `/odometry/filtered`

(Optionally, it may also inspect TF if implemented.)

---

## Frames (Canonical for Phase 1)

Use these frame names consistently across IMU, wheel odom, EKF, and downstream packages:

- **`odom`** — local odometry/world frame (continuous, drift allowed)
- **`base_link`** — robot base frame
- **`imu_link`** — IMU sensor frame

See also: [`docs/sign_conventions.md`](docs/sign_conventions.md)

---

## TF Ownership Rule (Important)

### EKF baseline ownership (recommended)

- **EKF (`robot_localization/ekf_node`) publishes TF**: `odom -> base_link`
- **Wheel odom node does NOT publish TF** when EKF is enabled

### Avoid this

- EKF and wheel odom publishing the same `odom -> base_link` transform at the same time

This causes duplicate TF publishers and unstable localization behavior.

---

## Launch File

### Main launch

- `launch/localization_bringup.launch.py`

This launch file should support toggles for:

- IMU node
- Wheel odom (C++)
- Wheel odom fallback (Python)
- EKF node
- Localization dashboard

---

## Example Launch Commands

> Adjust arguments to match the exact final implementation of `localization_bringup.launch.py`.

### 1) Raw validation mode (before EKF)

Use this to validate IMU + wheel odom first.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Savo_Pi/install/setup.bash

ros2 launch savo_localization localization_bringup.launch.py \
  use_imu:=true \
  use_wheel_odom:=true \
  use_ekf:=false \
  use_dashboard:=true
```

### 2) EKF baseline mode (Phase 1)

Use this after IMU and wheel odom are validated.

```bash
ros2 launch savo_localization localization_bringup.launch.py \
  use_imu:=true \
  use_wheel_odom:=true \
  use_ekf:=true \
  use_dashboard:=true
```

### 3) Fallback wheel odom mode (debug only)

Use Python fallback odom instead of the C++ node.

```bash
ros2 launch savo_localization localization_bringup.launch.py \
  use_imu:=true \
  use_wheel_odom:=false \
  use_wheel_odom_fallback:=true \
  use_ekf:=true \
  use_dashboard:=true
```

---

## Phase 1 Test Procedure (Professional Baseline)

### Step 0 — Build and source

```bash
cd ~/Savo_Pi
colcon build --packages-select savo_localization
source install/setup.bash
```

---

### Step 1 — Validate IMU (completed ✅)

#### Expected

- `/imu/data` publishes reliably
- `frame_id = imu_link` (or configured canonical IMU frame)
- angular velocity units are **rad/s**
- timestamps are valid (not stale)
- orientation/covariances are reasonable for EKF

#### Useful checks

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

---

### Step 2 — Validate wheel odometry (next step)

Run **raw validation mode** (EKF OFF).

#### Checks

```bash
ros2 topic hz /wheel/odom
ros2 topic echo /wheel/odom --once
```

#### Physical sign tests

1. **Forward motion**
   - Expect: `twist.twist.linear.x > 0`

2. **Reverse motion**
   - Expect: `twist.twist.linear.x < 0`

3. **Rotate CCW (left turn)**
   - Expect: `twist.twist.angular.z > 0`

4. **Rotate CW (right turn)**
   - Expect: `twist.twist.angular.z < 0`

#### Scale sanity checks

- Move approximately straight over a known distance (e.g., 1 m)
- Confirm odometry distance is reasonably close
- If wrong, tune:
  - wheel diameter/radius
  - encoder CPR / decoding / gear ratio
  - track width

---

### Step 3 — Bring up EKF (`robot_localization`)

Run **EKF baseline mode**.

#### Verify EKF output

```bash
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once
```

#### Verify TF

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

#### Optional TF graph snapshot

```bash
ros2 run tf2_tools view_frames
```

#### What to confirm

- `/odometry/filtered` is updating
- `odom -> base_link` TF is present and stable
- No duplicate TF publishers
- Robot standing still → low drift/noise only
- Forward/rotate signs remain correct after fusion

---

### Step 4 — Dashboard validation (recommended)

Run `localization_dashboard.py` to monitor:

- IMU freshness/rate
- Raw wheel odom values
- Fused EKF odom values
- Sign consistency
- Stale data alerts (if implemented)

Capture screenshots for documentation / team updates when useful.

---

## Configuration Files (Expected Roles)

- **`config/imu.yaml`**  
  IMU node parameters (I2C addr, frame, rates, covariances, etc.)

- **`config/encoders.yaml`**  
  Encoder and wheel odom parameters (pins, CPR, invert flags, wheel geometry)

- **`config/frames.yaml`**  
  Canonical frame names (`odom`, `base_link`, `imu_link`)

- **`config/localization_common.yaml`**  
  Shared params (rates, timeouts, debug/common settings)

- **`config/ekf_odom.yaml`**  
  `robot_localization` EKF config for the Phase 1 baseline

---

## Phase 1 Success Criteria

Phase 1 is considered successful when all of the following are true:

- ✅ `/imu/data` is valid and stable
- ✅ `/wheel/odom` is valid with correct signs/scales (within practical tolerance)
- ✅ `ekf_node` publishes `/odometry/filtered`
- ✅ TF `odom -> base_link` is present and stable
- ✅ Robot motion produces consistent fused odometry suitable for baseline localization/navigation testing

---

## Known Limitations (Current Baseline)

- Wheel odometry is based on **2 rear encoders only**
- Mecanum lateral motion (`vy`) is not fully observable with current encoder setup
- Fused localization is a **baseline approximation** and will improve later with:
  - 4 encoders (full mecanum odometry)
  - EKF tuning
  - optional IMU filtering
  - SLAM/AMCL integration in higher-level packages

---

## Future Upgrade Path

### Phase 2 (optional, after EKF baseline works)

- Compare raw IMU vs Madgwick-filtered IMU
- Tune EKF parameters only if needed

### Phase 3 (when 2 new encoders arrive)

- Upgrade wheel odometry to a 4-wheel mecanum model
- Re-test signs and scales
- Keep the same EKF pipeline
- Improve navigation/localization performance

---

## Notes for Developers

- Prefer **parameter tuning** before changing node code
- Keep frame names consistent across IMU, odom, EKF, and downstream packages
- Avoid duplicate TF publishers
- Document sign changes immediately in `docs/sign_conventions.md`

---

## Maintainer Workflow (Recommended)

1. Validate raw sensors/nodes first
2. Fuse with EKF
3. Confirm TF ownership
4. Capture evidence (`hz`, `echo`, dashboard screenshots)
5. Only then tune or refactor
