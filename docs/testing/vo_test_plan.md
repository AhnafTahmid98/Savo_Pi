# Visual Odometry Test Plan

## Objective

Verify Edge RGB-D odometry production, republishing, health, frames/timestamps/covariance and failure behavior before any Core EKF fusion is enabled.

## Scope

RGB-D sync/calibration inputs, C++ odometry, `/vo/odom/raw` and `/vo/odom`, status/quality/health, motion sanity, low-texture/low-light/dropout/restart, republisher and localization fusion gate.

## Test ownership

VO maintainer owns VO-001–004; Edge/localization integration team owns VO-005–009.

## Safety classification

VO-001–006 and VO-008–009 are `STATIC`, `UNIT`, `PC`, `HARDWARE-NON-ACTUATING`, `FAULT-INJECTION`, or `RECOVERY`; VO-007 motion collection is `HARDWARE-ACTUATING`/`GUARDED-FLOOR-MOTION` only after robot motion prerequisites pass.

## Preconditions

Validated D435 streams/mount/optical TF and time sync, control STOP for stationary tests, measured scene/course, C++ implementation selected, Core `use_vo=false` until VO-009 approval.

## Required hardware

Edge D435 and representative static/textured/low-texture/low-light scenes; guarded measured course for motion validation.

## Required software / configuration

VO C++ nodes, real-robot profile, covariance/health/republisher config, RealSense profile and optional Core VO EKF profile.

## Interfaces under test

RealSense aligned color/depth/info inputs; `/vo/odom/raw`, `/vo/odom`, status, health, tracking quality and diagnostics. VO owns no production map/base TF.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| VO-001 | T0 `STATIC` | Confirm C++ production default, input topics/QoS/delay, `vo_odom`/camera frames, output/republisher, jump/feature/covariance/health policy, and no map/base TF authority. |
| VO-002 | T1 `UNIT`/`PC` | Build/tests pass for sync, geometry/motion, frames/topics/params, covariance, quality/status, launch profiles, migration and republisher contracts. |
| VO-003 | T2 `TARGET-NON-HARDWARE` | Recorded/synthetic input produces finite monotonic odometry with expected frames and covariance; stale/misaligned/insufficient-feature/jump cases are unhealthy/rejected. |
| VO-004 | T3 `HARDWARE-NON-ACTUATING` | Stationary D435: measure input/output rate/latency, CPU, features/health, timestamp monotonicity, covariance and stationary drift over a recorded interval. |
| VO-005 | T3 `HARDWARE-NON-ACTUATING` | Test representative lighting/texture changes without robot motion; low light/texture-poor conditions degrade quality honestly and do not emit trusted jumps. |
| VO-006 | T6 `FAULT-INJECTION` | Stop/drop camera streams, desynchronize inputs, and recover; health becomes stale, republisher does not make old data current, and Core remains functional without VO. |
| VO-007 | T4/T5 `HARDWARE-ACTUATING` | On a separately authorized measured course, record translation and rotation compared with physical reference and wheel/IMU odometry. Acceptance thresholds must be approved; otherwise result remains BLOCKED after measurement. |
| VO-008 | T7 `RECOVERY` | Restart camera/VO/republisher; origin and output state follow contract, timestamps remain monotonic per session, no jump/stale replay reaches consumers. |
| VO-009 | T5 `INTEGRATION` | Only after VO-001–008 applicable PASS and review, explicitly enable Core VO EKF profile; verify covariance weighting, dropout fallback, health/readiness and TF remain correct. Disable fusion after test unless approved for release. |

## Pass criteria

Outputs are fresh/finite/monotonic with correct frames/covariance; health reflects scene/input quality; drift/motion meets approved criteria; dropout/restart is controlled; fusion remains opt-in and safe.

## Blocked criteria

D435/mount/time sync incomplete, course/acceptance thresholds absent, insufficient scene/privacy approval, or Core fusion review not approved.

## Failure criteria

Wrong frame/stamp/covariance, silent stale output, unbounded jump/drift, false health under dropout, republisher freshness inflation, or unapproved fusion.

## Abort criteria

STOP motion on localization/VO jump, safety loss, unexpected actuation/collision risk. Stop camera on thermal/USB instability or privacy breach.

## Evidence to retain

Commit/profiles/calibration/geometry, input/output rates/stamps/frames/covariance/quality, CPU/bandwidth, drift/course/lighting/texture measurements, bags/plots, dropout/restart and EKF comparison.

## Regression triggers

D435/profile/mount/calibration/QoS/time sync; VO algorithm/config/frames/jump/feature/covariance/health; republisher; localization EKF/VO enablement.

## Current validation status

Source tests exist. Current live rate/drift/motion/scene/dropout/restart and EKF fusion validation remain hardware/integration-dependent; fusion defaults off.

## Related documentation

- [VO package](../packages/savo_vo.md)
- [RealSense plan](realsense_test_plan.md)
- [Localization plan](localization_test_plan.md)
