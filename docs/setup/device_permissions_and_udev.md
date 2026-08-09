# Device permissions and udev

## Purpose

This guide establishes non-actuating access to Robot Savo devices using the target OS's least-privilege policy. It does not authorize writes to motors, servos, GPIO outputs, or safety hardware.

## Repository-supported policy

Current source explicitly checks that the LiDAR user belongs to `dialout`. Edge socket access is provisioned through `savomind-bridge` by `deploy/edge/prepare_runtime_sockets.sh`. The repository contains no general hardware udev rules and does not establish universal `video`, `audio`, `render`, `input`, I2C, or GPIO group assignments.

Therefore, inspect the installed host's device ownership before modifying membership. Record any site-managed udev or group policy outside the repository; do not invent a rule from a transient USB number.

## Detection and access inventory

```bash
id
groups
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
ls -l /dev/video* /dev/snd/* 2>/dev/null || true
ls -l /dev/i2c-1 /dev/gpiochip* 2>/dev/null || true
ls -l /dev/fb0 /dev/input/by-path/* 2>/dev/null || true
```

Use `/dev/i2c-1` only where current Robot Savo configuration calls for bus 1. Serial enumeration may be `/dev/ttyUSB*` or `/dev/ttyACM*`. The UI configuration currently names `/dev/fb0` and a board-specific path under `/dev/input/by-path`; verify the target rather than assuming either exists.

For one resolved device, record stable identity and path:

```bash
DEVICE=/dev/path-verified-on-this-host
stat -Lc '%A %U:%G %n' "$DEVICE"
udevadm info --query=property --name "$DEVICE"
```

`/dev/path-verified-on-this-host` is a deliberate target-specific value. Replace it with a path observed above; it is not a repository default.

## Device-specific requirements

| Device class | Source-backed requirement |
| --- | --- |
| LiDAR serial | Runtime user must be able to open the selected USB/ACM port; source checks `dialout` membership. |
| Core I2C | Bus 1 access for motor board, IMU, ToF mux, head, UPS, and ADC; host group/udev policy is not shipped. |
| Core GPIO | Access to the selected `/dev/gpiochip*` for encoders/ultrasonic; host group/udev policy is not shipped. |
| RealSense | USB/video access for the D435 and external RealSense driver; no repository udev rule is shipped. |
| Audio | ALSA capture/playback access to the stable configured PCM; no repository udev rule is shipped. |
| Display/touch | `/dev/fb0` and configured input path only when UI is enabled; no repository udev rule is shipped. |
| UPS/power | I2C bus 1 on both Pis; no separate character device is defined. |

The target administrator may add the deployment user to an existing OS-managed group only after confirming that group's ownership on the actual device. Re-login and repeat the access inventory. Do not use `chmod 777`, recursive world-write permissions, or broad wildcard rules.

## Verification and expected result

Run the non-actuating package checks in [RealSense](realsense_setup.md), [audio](audio_setup.md), and [UPS](ups_hat_setup.md). For Core serial/GPIO/I2C, detection and open permission may be commissioned; wheel direction, motor output, servo motion, ranging performance, and interrupt behavior remain formal hardware tests.

Expected setup result is stable device identification and access by the selected non-root runtime identity, with no global write access.

## Failure handling and evidence

On denial, compare the service user, current login groups, device owner/group/mode, udev properties, and systemd supplementary groups. Do not change several permission layers at once. Retain `id`, device listings, udev properties, installed site-policy reference, reboot re-enumeration result, and safe package-check output. Hardware authority is documented in the [bill of materials](../hardware/bill_of_materials.md) and [wiring overview](../hardware/wiring_overview.md).
