# Changelog

All notable changes to the Robot Savo repository should be recorded in this file.

The project does not yet use tagged semantic releases consistently. Entries are therefore grouped by date. This initial changelog was reconstructed from dated repository documents and the current source snapshot because the inspected exported ZIP does not contain Git history.

## 2026-08-06 — Documentation foundation and role truth

### Added

* Added the repository-level `README.md` with the distributed architecture, safety boundary, authoritative build entry points, production startup constraints, and documentation links.
* Added `docs/status/current_system_status.md` as the current, non-historical readiness record.
* Added this changelog.

### Changed

* Rewrote `docs/README.md` to define documentation scope, source-of-truth precedence, validation terminology, evidence requirements, and reading paths.
* Rewrote `docs/packages/package_ownership_matrix.md` from the actual package manifests and deployment role arrays.
* Documented the complete 20-package workspace and the exact 14-package Core and 10-package Edge build sets.
* Clarified that `savo_bridge` is an Edge production package, `savo_supervisor` is a Core production package, and `savo_observer` is built separately for the operator workstation.
* Clarified shared-package behavior for `savo_perception`, `savo_power`, `savo_bringup`, `savo_description`, and `savo_msgs`.

### Removed

* Removed the obsolete `docs/packages/savo_intent.md` page. There is no `savo_intent` ROS package in the current workspace; intent/LLM reasoning belongs to SavoMind and approved robot requests cross through `savo_bridge`.

### Validation

* `deploy/common/validate_full_bringup.sh`: PASS.
* `deploy/observer/validate_observer.sh`: PASS.
* `deploy/common/validate_pre_real_test_readiness.sh`: BLOCKED with no failed checks; Git metadata and `rosdep` were unavailable in the inspection environment.

## 2026-08-02 — Pre-real-test source closure

Source: `docs/audits/pre_real_test_completion_2026-08-02.md`.

### Added (pre-real-test source closure)

* Added persistent state backup/restore, integrity validation, overwrite protection, storage preflight, health checks, and log rotation.
* Added bounded speech protocol v2 transport between Robot Savo and SavoMind, including request/session correlation, validated TTS WAV return, peer credential checks, timeouts, and physical-playback acknowledgement.
* Added typed bridge adapters for authorized STOP, cancellation, bounded teleoperation, named-location navigation, mapping control/query, Scan360, save/verification/review/release observation, and supervisor queries.
* Added read-only UI integration for bringup, control, safety, navigation, speech, mapping, locations, and power state.
* Added Netplan/Chrony renderers, systemd rendering, role dependency installers, runtime socket preparation, and observer validation.
* Added the aggregate pre-real-test readiness validator.

### Safety (pre-real-test source closure)

* Preserved `STOP` as the production startup default.
* Preserved operator-only approval for map and semantic-location release.
* Preserved geometry and D435 hardware-validation gates.
* Confirmed that UI and observer paths remain read-only.
* Rejected generic ROS command forwarding, shell execution, motor bypass, supervisor bypass, and navigation-readiness bypass.

### Validation recorded by the audit

* 25 focused Robot source-contract tests passed.
* 362 SavoMind regression tests passed.
* Cross-repository speech protocol v2 smoke test passed.
* Backup/restore integration passed.
* Rendered systemd units passed verification.
* Netplan templates rendered without applying network changes.

## 2026-08-01 — Distributed bringup closure

Source: `docs/FULL_BRINGUP_CLOSURE_2026-08-01.md`.

### Added (distributed bringup closure)

* Completed shared distributed launch orchestration in `savo_bringup`.
* Added role-selecting `robot_bringup.launch.py` for Core and Edge.
* Added supported robot modes: `safe_idle`, `manual`, `manual_mapping`, `autonomous_mapping`, `saved_map_navigation`, and `diagnostics`.
* Added bringup profiles: `bench`, `lidar_only`, `lidar_d435_voxel`, and `production`.
* Added independent Core and Edge readiness state, ready, heartbeat, and diagnostic outputs.

### Safety (distributed bringup closure)

* Required locked geometry for motion-capable non-bench profiles.
* Prohibited provisional geometry in production.
* Required explicit D435 voxel validation before selecting the voxel profile.
* Kept saved-map navigation behind the verified AM-8 release path.
* Kept autonomous mapping behind `RunAutonomousMapping` contract v2 and quality/review gates.
* Kept Core control startup in `STOP`.

## Maintenance policy

Future entries should include:

* date and release/tag when available;
* affected packages and deployment roles;
* added, changed, fixed, removed, security, and safety sections as applicable;
* migration or rollback notes;
* validation environment and retained evidence;
* hardware revalidation requirements caused by the change.

Do not use this changelog as the only validation record. Detailed commands, outputs, profiles, and pass/fail evidence belong in the applicable test log or dated audit.
