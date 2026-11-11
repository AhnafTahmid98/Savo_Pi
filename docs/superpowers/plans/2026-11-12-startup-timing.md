# Startup timing investigation implementation plan

**Goal:** Explain the supplied Pi startup failure and add bounded timing evidence and regression coverage.
**Spec:** User request in this session, diagnostic bundle `~/savo-diagnostics/savo_startup_20261112_004950/`.
**Architecture:** Add observational counters to the existing single-threaded localization consumer and startup coordinator. Keep evaluation, confirmation, latching, freshness and arming policy unchanged. Use existing health/status outputs plus a compact startup trace stream.
**Tech stack:** C++17, ROS 2 Jazzy, gtest, pytest; isolated PC Docker build.

## Constraints and design decisions

- Work directly on master; preserve 5385fde and 0202e86. No commits or pushes.
- No EKF frequency, freshness, motor safety, executor or recovery-policy changes.
- Prefer instrumentation over a speculative executor/DDS/scheduler fix: the capture lacks callback and coordinator receive ordering.
- Monotonic intervals, fixed-size last/max counters, no per-sensor logging. CPU duration separates executing work from elapsed blocking/preemption; it cannot distinguish blocking from scheduler delay alone.
- DDS metadata is observational, optional and clock-dependent; do not use it for readiness.
- Completed evaluation durations are reported on the next health sample, with their sequence identifier.
- Coordinator trace distinguishes evaluation count from received health samples, records pre/post confirmation, recovery eligibility and terminal decision. It never changes those decisions.

## Work

- [x] Correlate launch, independent topic timestamps and scheduler coverage; document evidence limits.
- [x] Add tests for timing accumulation and optional timestamps before implementation; add bounded callback/evaluation instrumentation.
- [x] Add dependency-confirmation counts and compact trace, retaining exact matching behavior; test transient reset, changed signatures, repeated cached failure and terminal recovery ordering.
- [x] Extend localization/safety regressions for stale data, TF, invalid frames/data/timestamps and pose faults; prove no added grace.
- [x] Build savo_msgs, savo_localization and savo_bringup in Jazzy on PC; run relevant full package suites, inspect diff and report Pi-only validation.

## Review focus

Missing DDS timestamps; ROS/system clock mismatches; recovered health arriving after terminal failure; repeated evaluation of one report; multiple simultaneous safety faults. Preserve startup failure latching. Hardware timing cannot be reproduced by deterministic PC tests.

## Outcome

All planned implementation and PC validation completed. See `docs/diagnostics/2026-11-12-autonomous-mapping-startup-timing.md` for evidence, exact checks and remaining Pi-only uncertainties. Final package results: 805 tests, zero errors/failures, five cppcheck skips. An independent read-only review found no Critical/Important issues; both minor diagnostic/test findings were addressed and the full suites rerun. No commits or pushes were made. The underlying Pi timing cause remains unproven, so no speculative runtime/safety policy fix was introduced.
