# savo_perception (Robot Savo)

Perception + safety package for Robot Savo (ROS 2 Jazzy).  
This package is **hybrid**:
- **Python nodes** publish simple perception signals (ranges, depth front-min, safety decision).
- A **C++ safety gate** enforces safe velocity output with low latency and stable timing.

Primary goal: provide a reliable reflex layer so the robot **slows down and stops** before collisions, even if Nav2/costmaps lag.

---

## 1) Package responsibilities

### A) Range sensing (near-field)
Publishes near-field distances from:
- **VL53L1X ToF left and right** (side protection)
- **Ultrasonic (front center)** (backup for front)
- **RealSense D435** provides the primary front obstacle signal via `/depth/min_front_m`

### B) Depth-based front obstacle distance (D435)
Computes a robust **front obstacle distance** from the RealSense depth stream
(ROI + percentile approach) and publishes a single float topic.

### C) Safety decision (STOP + optional slowdown)
Combines **Depth front-min + Ultrasonic + Side ToFs** into:
- `/safety/stop` (hard stop)
- optional `/safety/slowdown_factor` (0.0–1.0 scaling)

### D) Safety gate (C++)
Applies STOP/slowdown to velocity commands:
- `/cmd_vel` -> `/cmd_vel_safe`

---

## 2) Node inventory

Python nodes live under: `savo_perception/nodes/`  
Low-level sensor APIs live under: `savo_perception/sensors_api/`

### 2.1 vl53_node.py
**Purpose:** publish side ToF distances (left/right).

**Publishes**
- `/savo_perception/range/left_m`  (std_msgs/Float32)
- `/savo_perception/range/right_m` (std_msgs/Float32)

**Parameters** (see `config/range_safety.yaml`)
- `tof_publish_hz`
- `tof_filter_window`
- `stale_timeout_s`

---

### 2.2 ultrasonic_node.py
**Purpose:** publish ultrasonic distance (front backup).

**Publishes**
- `/savo_perception/range/front_ultrasonic_m` (std_msgs/Float32)

**Parameters** (see `config/range_safety.yaml`)
- `ultrasonic_publish_hz`
- `ultrasonic_filter_window`
- `stale_timeout_s`

---

### 2.3 depth_front_min_node.py  (RealSense D435)
**Purpose:** compute robust front obstacle distance from depth ROI.

**Subscribes**
- `/camera/camera/depth/image_rect_raw` (sensor_msgs/Image)  
  *(topic may differ depending on realsense2_camera config; adjust in launch if needed)*

**Publishes**
- `/depth/min_front_m` (std_msgs/Float32)

**Parameters** (see `config/depth_front.yaml`)
- `roi_x`, `roi_y`, `roi_w`, `roi_h` (pixel ROI on depth image)
- `percentile` (e.g., 10th percentile distance in ROI)
- `min_valid_m`, `max_valid_m`
- `filter_window`
- `publish_hz`

---

### 2.4 safety_stop_node.py
**Purpose:** decide STOP / slowdown based on near-field + depth front-min.

**Subscribes**
- `/savo_perception/range/left_m`
- `/savo_perception/range/right_m`
- `/savo_perception/range/front_ultrasonic_m`
- `/depth/min_front_m`

**Publishes**
- `/safety/stop` (std_msgs/Bool)
- `/safety/slowdown_factor` (std_msgs/Float32) *(optional, recommended)*
- `/safety/reason` (std_msgs/String) *(optional for debugging)*

**Parameters** (see `config/range_safety.yaml` + `config/depth_front.yaml`)
- Front: `front.stop_m`, `front.slow_m`, `front.*debounce*`, `front.hysteresis_m`
- Side:  `side.stop_m`, `side.slow_m`, `side.global_stop_on_side`, `side.*debounce*`
- Global: `fail_safe_on_stale`, `stale_timeout_s`, `min_valid_m`, `max_valid_m`
- Slowdown: `slowdown.publish_global_factor`, `slowdown.min_global_scale`, `slowdown.ema_alpha`

---

### 2.5 cmd_vel_safety_gate (C++) — `src/cmd_vel_safety_gate.cpp`
**Purpose:** enforce safety on velocity commands with stable timing.

**Subscribes**
- `/cmd_vel` (geometry_msgs/Twist)
- `/safety/stop` (std_msgs/Bool)
- `/safety/slowdown_factor` (std_msgs/Float32) *(optional)*

**Publishes**
- `/cmd_vel_safe` (geometry_msgs/Twist)

**Parameters**
- `publish_hz` (recommended 50–100 Hz)
- `zero_on_timeout_ms`
- `max_scale_min` (minimum allowed scale, if used)

---

## 3) Launch files

Launch directory: `launch/`

### 3.1 realsense_bringup.launch.py
Starts the RealSense driver (realsense2_camera) and applies parameters/remaps.
This package does not replace `realsense2_camera`—it wraps it for consistent bringup.

### 3.2 perception_bringup.launch.py
Starts:
- ToF + ultrasonic publishers
- depth front-min node
- safety stop decision node
- C++ cmd_vel safety gate

### 3.3 safety_bringup.launch.py
Starts only:
- safety decision node
- C++ cmd_vel safety gate  
Useful for testing the safety pipeline with Nav2/teleop already running.

---

## 4) Nav2 integration (D435 3D obstacle layers)

Configuration file:
- `config/costmap_3d_layers.yaml`

This contains parameters for Nav2 costmap plugins such as:
- **Voxel Layer** OR **Spatio-Temporal Voxel Layer (STVL)**

RealSense pointcloud (typical):
- `/camera/camera/depth/color/points` (sensor_msgs/PointCloud2)

Important:
- Costmap layers help global/local avoidance,
- but the **last-second braking** must rely on `/safety/stop` + `/cmd_vel_safe`.

---

## 5) Topic contract summary (LOCKED)

### Inputs
- `/camera/camera/depth/image_rect_raw` (depth image)
- `/cmd_vel` (from Nav2/teleop)

### Outputs
- `/depth/min_front_m`
- `/savo_perception/range/left_m`
- `/savo_perception/range/right_m`
- `/savo_perception/range/front_ultrasonic_m`
- `/safety/stop`
- `/safety/slowdown_factor` (optional)
- `/cmd_vel_safe`

---

## 6) Notes / engineering rules (Robot Savo)
- **Front stop threshold** target: ~0.28 m (Depth + Ultrasonic safety).
- **Side stop threshold** target: ~0.20 m (ToF side protection).
- If STOP triggers: robot halts immediately; higher-level logic (Nav2 recoveries in `savo_nav`) may later reverse + re-plan.
- C++ safety gate is used for determinism and reduced jitter (prevents “late stop” collisions).
- Depth/voxel layers are complementary to near-field safety; they improve avoidance but are not a substitute for the reflex stop layer.

---
