# Savo Core fresh installation

## Purpose and target

This procedure prepares the Raspberry Pi 5 that owns drivetrain execution, safety perception, localization, mapping/navigation, locations, head hardware, and supervision. The supported target is Ubuntu 24.04 LTS ARM64 with ROS 2 Jazzy and hostname `core` or `savo-core`.

## Prerequisites

- Hardware is wired against the [GPIO/I2C map](../hardware/gpio_i2c_map.md), with emergency stop available.
- The robot is secured against motion and control remains `STOP`.
- ROS Jazzy exists at `/opt/ros/jazzy`.
- The authorized repository is checked out at `$HOME/Savo_Pi`, or `SAVO_ROOT` is set to its absolute path.
- The deployment user is non-root and will consistently own the runtime.

```bash
cd "$HOME/Savo_Pi"
hostname
uname -m
grep '^VERSION_ID=' /etc/os-release
git status --short
git rev-parse HEAD
```

Do not proceed if the hostname, OS, architecture, or revision is wrong.

## Install dependencies

```bash
cd "$HOME/Savo_Pi"
bash deploy/common/install_role_deps.sh --role core
```

The installer checks Ubuntu 24.04, ARM64, free space, ROS, and the exact 14-package Core array; installs common/Core APT requirements; runs role-scoped `rosdep`; builds the role; and retains a log. Resolve its errors rather than manually broadening the role.

## Prepare persistent storage

```bash
sudo bash deploy/core/prepare_runtime_storage.sh \
  --owner "$USER" \
  --group "$USER"
```

This creates the persistent state hierarchy under `/var/lib/robot_savo` and logs under `/var/log/robot_savo`, owned by the selected runtime identity with mode `0750`. A new installation is expected to contain empty databases/artifact directories. These paths survive reboot; `/tmp` and `/run` do not.

```bash
stat -c '%A %U:%G %n' /var/lib/robot_savo /var/log/robot_savo
find /var/lib/robot_savo -maxdepth 2 -type d -print
```

Back up persistent state using the [backup procedure](../operations/backup_restore_and_rollback.md) before later upgrades; never substitute a temporary path for production state.

## Verify hardware access

Core source uses I2C bus 1 for the motor board, IMU, ToF mux, head hardware, power monitor, and ADC; GPIO chips for encoders/ultrasonic; and serial USB/ACM devices for LiDAR. The repository explicitly checks `dialout` for serial LiDAR access. It does not ship general I2C/GPIO udev rules or establish a universal group for those devices.

```bash
id
ls -l /dev/i2c-1 /dev/gpiochip* 2>/dev/null || true
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
```

Verify access using the installed host policy and [device-permission guide](device_permissions_and_udev.md). Do not issue motor, servo, or GPIO-output commands. Camera, power, and address details are in the [hardware registry](../hardware/bill_of_materials.md) and [power setup](ups_hat_setup.md).

## Build and validate

```bash
cd "$HOME/Savo_Pi"
bash deploy/core/build_core.sh --clean --test
bash deploy/common/validate_full_bringup.sh
```

Required evidence is all 14 Core packages present, a clean build, and zero test failures/errors. A missing-package development bypass is not acceptable for commissioning.

## Configure the service

Prepare the protected environment as described in [environment and secrets](environment_and_secrets.md), setting `SAVO_ROLE=core`. Render and review the units:

```bash
sudo bash deploy/systemd/render_units.sh \
  --user "$USER" \
  --group "$USER" \
  --root "$PWD" \
  --output-dir /tmp/robot-savo-units
systemd-analyze verify /tmp/robot-savo-units/savo_core.service
sudo install -m 0644 /tmp/robot-savo-units/savo_core.service \
  /etc/systemd/system/savo_core.service
sudo systemctl daemon-reload
```

Do not enable both `savo_core.service` and generic `savo.service` in Core mode. Rendering and enabling are separate from starting.

## Safe-idle commissioning

First start interactively:

```bash
cd "$HOME/Savo_Pi"
bash deploy/core/run_core.sh
```

Confirm the launcher reports Core, `safe_idle`, `lidar_only`, control `STOP`, locked geometry required, provisional geometry rejected, D435 voxel validation false, and speech/UI disabled. From another correctly configured shell inspect without publishing commands:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
ros2 node list
ros2 topic list
```

Verify supervisor/readiness/state and TF are visible, devices do not report unexpected ownership, and there is no motor output or motion. Geometry lock may deliberately block motion; that is an expected safe result, not a setup failure.

Only after interactive safe idle passes:

```bash
sudo systemctl enable savo_core.service
sudo systemctl start savo_core.service
systemctl status savo_core.service --no-pager
journalctl -u savo_core.service -b --no-pager
```

## Failure handling, evidence, and next step

For wrong hostname, installer/`rosdep` errors, device denial, or storage ownership, stop and fix that prerequisite. For a restart loop, stop the unit before diagnosis. Retain revision, installer/build/test logs, directory/device listings, effective environment, and the first-boot journal. Continue with the [commissioning checklist](commissioning_checklist.md), not a motion command. Deployment detail is in [Core deployment](../deployment/deploy_savo_core.md).
