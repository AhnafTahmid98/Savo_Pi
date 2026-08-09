# Robot Savo documentation

This directory contains the current system-integration, deployment, hardware, package, and validation documentation for the Robot Savo ROS 2 Jazzy workspace.

## Source of truth

When records disagree, use this order:

1. current source and ROS interface definitions;
2. current launch/configuration;
3. manifests and build/install rules;
4. deployment role arrays and scripts;
5. validators and tests;
6. current status and ownership documents;
7. package-local READMEs;
8. older architecture, test, audit, and historical records.

An earlier Robot Savo source baseline was exercised on physical hardware. That is valuable baseline evidence, but current source/configuration changes still require regression. Source validation is not motion authorization.

## Start here

- [Repository overview](../README.md)
- [Current system status](status/current_system_status.md)
- [Package ownership matrix](packages/package_ownership_matrix.md)
- [System overview](architecture/system_overview.md)
- [Two-Pi architecture](architecture/two_pi_architecture.md)
- [Production startup](deployment/production_startup.md)
- [Full robot test plan](testing/full_robot_test_plan.md)

## Package documentation

Exactly one central page exists for each of the 20 ROS packages:

| Package | Primary deployment | Documentation |
| --- | --- | --- |
| `savo_base` | Core | [Base](packages/savo_base.md) |
| `savo_bringup` | Core and Edge | [Bringup](packages/savo_bringup.md) |
| `savo_bridge` | Edge | [Bridge](packages/savo_bridge.md) |
| `savo_control` | Core | [Control](packages/savo_control.md) |
| `savo_description` | Core, Edge, observer dependency | [Description](packages/savo_description.md) |
| `savo_head` | Core | [Head](packages/savo_head.md) |
| `savo_lidar` | Core | [LiDAR](packages/savo_lidar.md) |
| `savo_localization` | Core | [Localization](packages/savo_localization.md) |
| `savo_locations` | Core | [Locations](packages/savo_locations.md) |
| `savo_mapping` | Core | [Mapping](packages/savo_mapping.md) |
| `savo_msgs` | Interface dependency | [Messages/interfaces](packages/savo_msgs.md) |
| `savo_nav` | Core | [Navigation](packages/savo_nav.md) |
| `savo_observer` | Operator workstation | [Observer](packages/savo_observer.md) |
| `savo_perception` | Core and optional Edge path | [Perception](packages/savo_perception.md) |
| `savo_power` | Core and Edge | [Power](packages/savo_power.md) |
| `savo_realsense` | Edge | [RealSense](packages/savo_realsense.md) |
| `savo_speech` | Edge, optional startup | [Speech](packages/savo_speech.md) |
| `savo_supervisor` | Core | [Supervisor](packages/savo_supervisor.md) |
| `savo_ui` | Edge, optional startup | [UI](packages/savo_ui.md) |
| `savo_vo` | Edge | [Visual odometry](packages/savo_vo.md) |

There is no `savo_intent` package or central page. SavoMind reasoning is external and approved typed operations cross `savo_bridge`.

## Architecture

- [System overview](architecture/system_overview.md)
- [Two-Pi architecture](architecture/two_pi_architecture.md)
- [Core architecture](architecture/savo_core_architecture.md)
- [Edge architecture](architecture/savo_edge_architecture.md)
- [Shared packages](architecture/shared_packages_architecture.md)
- [Bringup readiness state machine](architecture/bringup_readiness_state_machine.md)
- [Motion authority](architecture/motion_authority_model.md)
- [Safety](architecture/safety_architecture.md)
- [Localization](architecture/localization_architecture.md)
- [Mapping and navigation](architecture/mapping_navigation_architecture.md)
- [Perception](architecture/perception_architecture.md)
- [ROS topic contracts](architecture/ros2_topic_contracts.md)
- [TF frame authority](architecture/tf_frame_authority.md)
- [Network](architecture/network_architecture.md)
- [Speech/SavoMind flow](architecture/speech_intent_flow.md)
- [SavoMind–ROS boundary](architecture/savomind_ros_boundary.md)
- [Data storage and artifacts](architecture/data_storage_and_artifacts.md)
- [Diagnostics and observability](architecture/diagnostics_and_observability.md)

## Deployment and operations

- [Role-based builds](deployment/role_based_builds.md)
- [Core deployment](deployment/deploy_savo_core.md)
- [Edge deployment](deployment/deploy_savo_edge.md)
- [Systemd services](deployment/systemd_services.md)
- [Release checklist](deployment/release_checklist.md)
- [Recovery operations](deployment/recovery_operations.md)

## Hardware and setup

- [Bill of materials](hardware/bill_of_materials.md)
- [Wiring overview](hardware/wiring_overview.md)
- [Cable and connector map](hardware/cable_and_connector_map.md)
- [GPIO/I2C map](hardware/gpio_i2c_map.md)
- [Measurement checklist](hardware/measurement_checklist.md)
- [Calibration register](hardware/calibration_register.md)
- [Mechanical notes](hardware/mechanical_notes.md)
- [Robot layer layout](hardware/robot_layer_layout.md)
- [Sensor mounting](hardware/sensor_mounting.md)
- [Power architecture](hardware/power_architecture.md)
- [Hardware revision history](hardware/hardware_revision_history.md)
- [Core setup](setup/savo_core_setup.md)
- [Edge setup](setup/savo_edge_setup.md)
- [Dependency matrix](setup/dependency_matrix.md)
- [Core–Edge Ethernet](setup/ethernet_core_edge_setup.md)
- [ROS networking](setup/ros_domain_networking.md)
- [Time synchronization](setup/time_sync.md)
- [RealSense setup](setup/realsense_setup.md)
- [Audio setup](setup/audio_setup.md)

## Testing and evidence

- [Component validation overview](testing/component_validation_overview.md)
- [Core component validation](testing/savo_core_component_validation.md)
- [Edge component validation](testing/savo_edge_component_validation.md)
- [Real-robot acceptance checklist](testing/real_robot_acceptance_checklist.md)
- [Pre-real-test completion audit](audits/pre_real_test_completion_2026-08-02.md)

Package-specific test plans are linked from each package page.

## Validation terminology

- **Implemented:** source/config/interface exists.
- **Source-validated:** static/source-contract checks passed.
- **PC-validated:** build/tests ran successfully on a development PC.
- **Target-validated:** build/tests ran on the intended Core, Edge, or observer host.
- **Hardware-validated:** behavior was exercised on physical hardware.
- **Integration-validated:** multiple production components were tested together.
- **Blocked:** a required external dependency, measurement, environment, authorization, or earlier gate is missing.
- **Deferred:** intentionally outside current production scope.

Retain exact commands, environment, revision, profile, result, and evidence location for validation claims.
