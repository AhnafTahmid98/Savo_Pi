# Changelog

All notable changes to the Robot Savo repository should be recorded in this file.

The project does not yet use tagged semantic releases consistently. Entries are therefore grouped by date. This initial changelog was reconstructed from dated repository documents and the current source snapshot because the inspected exported ZIP does not contain Git history.

## 2026-08-09 — Phase 5 operations documentation

### Added — production operations runbooks

* Added the 16-document production operations set covering the operator quick
  start, pre-operation inspection, startup/shutdown, emergency response,
  manual drive, mapping, navigation, speech/UI, and map/location administration.
* Added source-backed log collection, symptom-oriented troubleshooting,
  maintenance, backup/restore/rollback, incident response, and a directly
  usable incident report template.

### Changed — operational safety and recovery guidance

* Formalized motion warnings, STOP/abort criteria, safe-idle/readiness checks,
  evidence preservation, role boundaries, and explicit escalation rules.
* Reconciled backup and restore examples with their required current flags and
  kept internal motion, Nav2, release, SQLite, and bridge bypasses out of
  routine operator procedures.
* Documented only installed command-line surfaces; unsupported operator
  clients remain explicitly identified instead of being invented.

### Safety and behavior

* No motion authority, safety gate, configuration, or production runtime
  behavior was changed by Phase 5 documentation.

## 2026-08-09 — Phase 4 architecture and hardware documentation

### Added — architecture and hardware registries

* Added bringup-readiness, SavoMind boundary, TF-authority, persistent-storage, and diagnostics architecture pages, completing the 18-document architecture set.
* Added a source-derived bill of materials, calibration register, cable/connector map, and hardware revision history, completing the 11-document hardware set.

### Changed — source reconciliation

* Rewrote the existing architecture pages around the current two-Pi roles, fail-closed motion chain, permission-versus-execution boundary, distributed interfaces, storage, degraded behavior, and validation gates.
* Rewrote the existing hardware pages as serviceable engineering records, explicitly distinguishing source configuration from physical measurement and historical evidence.
* Centralized TF ownership and documented the mode-exclusive `map -> odom`, EKF-owned `odom -> base_footprint`, URDF fixed-frame, disabled RealSense TF, and uncalibrated/disabled head-TF contracts.

### Safety and production findings

* Recorded the mismatch between configured `0.165/0.165 m` kinematic wheelbase/track and the provisional URDF wheel-centre layout implying `0.230/0.200 m`; no software value was guessed or changed.
* Recorded that base and head drivers share and initialize the same Core PCA9685 at `0x40`; channel allocation is non-overlapping but startup/reset/concurrency requires target validation.
* Preserved provisional-geometry, D435 voxel, operator-approval, Core motion-authority, UI/observer read-only, and hardware-regression gates.

### Validation — Phase 4

* Confirmed exact documentation coverage: 18 architecture pages and 11 hardware pages.
* Checked Markdown links, placeholders/stale package references, whitespace, role/bringup validators, and aggregate pre-real-test readiness; detailed results are reported in the Phase 4 handoff.

## 2026-08-09 — Phase 3 package documentation

### Added — central package pages

* Completed one source-reconciled central integration page for each of the 20 real ROS 2 packages under `docs/packages/`.
* Added the previously missing pages for bridge, head, LiDAR, locations, messages, observer, power, RealSense, and supervisor.

### Changed — package documentation reconciliation

* Replaced package placeholders and the contaminated visual-odometry page with implementation-specific deployment, node, interface, TF, safety, persistence, build, and validation documentation.
* Reconciled stale package claims against current C++/Python authority paths, launch/configuration, deployment arrays, interface definitions, and validators.
* Normalized Core/Edge/observer ownership and permission-versus-execution boundaries; preserved operator-only release/location approval and the fail-closed motion chain.
* Restored `docs/README.md` from a concatenated file dump to a navigable documentation index.
* Corrected the non-package `future/` staging-tree description and recorded the current optional UI v2 workspace path without changing its classic production default.

### Validation — Phase 3

* Confirmed exact coverage: 20 package manifests and 20 matching central pages, with no `savo_intent` page.
* Checked repository Markdown relative links and required source validators; detailed results are reported with the Phase 3 handoff.
* Ran `git diff --check` and audited all package pages for interface, authority, TF, deployment, and validation terminology consistency.

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

### Validation — 2026-08-06 inspection

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
