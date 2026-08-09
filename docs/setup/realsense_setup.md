# Intel RealSense D435 setup

## Purpose and target

This guide commissions the Edge-owned Intel RealSense D435 for detection, USB 3 operation, configured streams, health, and VO input. It does not validate VO accuracy or the separately gated D435 obstacle-cloud path.

## Prerequisites

- Edge dependencies and the 10-package build pass.
- No other process owns the camera.
- The D435 is connected to a USB 3-capable port/cable.
- The deployment user has target-supported USB/video access.
- Core remains in `STOP` and Edge uses safe idle.

The ROS camera driver is the external `realsense2_camera` package. Firmware changes are outside repository authority; record the detected firmware and use the approved hardware baseline rather than upgrading during commissioning.

## Detection and binding

```bash
rs-enumerate-devices
lsusb -t
```

Confirm a D435 is present and negotiated at SuperSpeed rather than USB 2. The current default Edge camera configuration binds serial `801212070967`. Verify that physical serial before launch. A replacement camera with a different serial will not satisfy the default binding; make any binding change as a reviewed configuration change and retain the old/new serial evidence.

## Current stream contract

The default `realsense_d435_camera.yaml` enables:

| Stream/setting | Value |
| --- | --- |
| Color | `640x480x30` |
| Depth | `848x480x30` |
| Aligned depth | enabled |
| Synchronization | enabled |
| Point cloud in camera driver | enabled |
| Camera TF publication | disabled; `savo_description` owns fixed TF |

Required image topics are `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`, `/camera/camera/depth/image_rect_raw`, and `/camera/camera/depth/camera_info`. Camera status is `/realsense/status`. VO uses these RGB-D inputs and reports `/vo/odom`, `/vo/status`, and `/vo/health`.

## Safe launch and verification

Use the role runner so ownership and defaults match production:

```bash
cd "$HOME/Savo_Pi"
bash deploy/edge/run_edge.sh
```

From a second sourced shell:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
ros2 run savo_realsense realsense_smoke_test_cli
ros2 topic echo /realsense/status --once
ros2 topic echo /vo/status --once
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

Expected result: the smoke test finds an Intel USB device and all required topics, status is current, streams approach their configured 30 Hz under stable load, and VO progresses beyond missing-input errors. Waiting for a reference or lack of scene texture is not proof of VO performance.

## Obstacle-cloud gate

Edge bringup defaults the D435 obstacle cloud off and `SAVO_D435_VOXEL_VALIDATED=false`. Keep both states during setup. Camera operation does not validate self-filter bounds, transforms, voxel/filter tuning, dropout behavior, or Nav2 interaction. Those require the [perception test plan](../testing/perception_test_plan.md).

## Failure handling and evidence

- Missing device: check power, cable, port, `rs-enumerate-devices`, and kernel USB listing.
- USB 2 fallback: replace/reseat the approved SuperSpeed cable/port; do not lower stream requirements and call that acceptance.
- Serial mismatch: confirm the installed unit and review the configuration change.
- Permission denied: use [device permissions](device_permissions_and_udev.md); do not make video devices world-writable.
- Unstable/missing streams: stop duplicate camera owners, inspect Edge journal/USB topology, then restart the single owner.

Retain model/serial/firmware output, USB topology, effective config, smoke test, topic rates/status, and journal. Continue with [VO testing](../testing/vo_test_plan.md), not motion.
