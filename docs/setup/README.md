# Setup and commissioning

## Purpose

This is the entry point for taking fresh Robot Savo hosts through installation and non-actuating, safe-idle commissioning. Setup establishes a reproducible host; deployment installs its runtime service; commissioning proves that the installed roles can start safely. Formal hardware and motion acceptance follows later.

Commissioning never authorizes motor output, `MANUAL`, navigation, or autonomous mapping. A provisional geometry profile may still permit setup to pass while motion remains blocked.

## Supported systems

| Guide | Target |
| --- | --- |
| [Core setup](savo_core_setup.md) | Raspberry Pi 5, Ubuntu 24.04 LTS, ROS 2 Jazzy, hostname `core` or `savo-core` |
| [Edge setup](savo_edge_setup.md) | Raspberry Pi 5, Ubuntu 24.04 LTS, ROS 2 Jazzy, hostname `edge` or `savo-edge` |
| [Observer setup](observer_pc_setup.md) | Ubuntu 24.04 / ROS 2 Jazzy workstation |
| [SavoMind integration](savomind_edge_setup.md) | Robot Savo side of the Edge-local integration |

The role dependency installer rejects a non-ARM development computer unless `SAVO_ALLOW_PC_INSTALL=true`; that override is for development and is not target commissioning evidence.

## Required order

```text
Fresh OS
   ↓
Development/system prerequisites
   ↓
ROS 2 Jazzy
   ↓
Repository checkout
   ↓
Role dependencies
   ↓
Network/time
   ↓
Permissions
   ↓
Persistent/runtime storage
   ↓
Hardware-specific setup
   ↓
Build
   ↓
Safe-idle commissioning
   ↓
Formal test phase
```

For an existing installation, record the current revision and configuration, then begin at the first changed or unverified stage. Do not treat an old build or another host's install tree as evidence.

## Installation paths

### Core installation

1. [First-installation checklist](first_installation_checklist.md)
2. [Development environment](development_environment.md)
3. [Core setup](savo_core_setup.md)
4. [Network and time](network_and_time_setup.md)
5. [Device permissions](device_permissions_and_udev.md)
6. [UPS HAT setup](ups_hat_setup.md)
7. [Commissioning checklist](commissioning_checklist.md)

### Edge installation

1. [First-installation checklist](first_installation_checklist.md)
2. [Development environment](development_environment.md)
3. [Edge setup](savo_edge_setup.md)
4. [Network and time](network_and_time_setup.md)
5. [Device permissions](device_permissions_and_udev.md)
6. [RealSense setup](realsense_setup.md)
7. [Audio setup](audio_setup.md)
8. [SavoMind integration](savomind_edge_setup.md)
9. [Commissioning checklist](commissioning_checklist.md)

### Observer workstation

1. [Development environment](development_environment.md)
2. [Observer setup](observer_pc_setup.md)
3. [ROS domain networking](ros_domain_networking.md)
4. [Commissioning checklist](commissioning_checklist.md)

## Complete setup set

- [First-installation checklist](first_installation_checklist.md)
- [Core setup](savo_core_setup.md)
- [Edge setup](savo_edge_setup.md)
- [Observer setup](observer_pc_setup.md)
- [SavoMind Edge setup](savomind_edge_setup.md)
- [ROS domain networking](ros_domain_networking.md)
- [Network and time](network_and_time_setup.md)
- [Device permissions and udev](device_permissions_and_udev.md)
- [Environment and secrets](environment_and_secrets.md)
- [RealSense setup](realsense_setup.md)
- [Audio setup](audio_setup.md)
- [UPS HAT setup](ups_hat_setup.md)
- [Development environment](development_environment.md)
- [Commissioning checklist](commissioning_checklist.md)

## Evidence and next step

Retain the host identities, commit, rendered configuration, installer/build/test logs, device listings, clock/discovery output, and the signed commissioning checklist. A passing checklist means only “ready to begin formal validation.” Continue with the [component validation overview](../testing/component_validation_overview.md) and [real-robot acceptance checklist](../testing/real_robot_acceptance_checklist.md).

See also the [current system status](../status/current_system_status.md), [role-based builds](../deployment/role_based_builds.md), and [production startup](../deployment/production_startup.md).
