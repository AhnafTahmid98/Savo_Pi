# Mapping-local authority implementation plan

> **For agentic workers:** Use superpowers:executing-plans or superpowers:subagent-driven-development and TDD for each task.

**Goal:** Autonomous mapping operates without any running system Supervisor.

**Architecture:** The existing mapping supervisor observes subsystem health directly and publishes a mapping-local readiness contract. The autonomous orchestrator alone admits a mission, owns its local authority and NAV transitions, and revokes on genuine dependency loss. Child Coverage/semantic operations validate the active mapping mission; standalone system-supervised operations remain supported.

**Tech Stack:** C++17, ROS 2 Jazzy/FastDDS, existing JSON subsystem observations, typed ROS actions/services, steady-clock receive freshness.

**Spec:** User-approved ownership design and fourteen implementation constraints in the 2026-09-14 conversation. This file records the implementation boundaries and validation; approval has already been given.

## Global constraints

- Preserve commit `5081dc2`; no changes to system Supervisor behavior.
- No hardware, TF ownership, velocity routing, safety threshold or algorithm changes.
- `/cmd_vel_safe` remains the physical gate; launch remains STOP with no automatic action.
- Base battery/Core UPS required; Edge UPS optional unless explicitly expected.
- Current freshness contracts remain unchanged; invalid input and regression fail closed.
- Local authority binds mission/request/actor/map/revision/semantic scope/generation.
- Environmental STOP permits existing healthy missions to continue; new admission requires clear safety.
- No automatic re-arm/resume after genuine failure. Terminal cleanup commands STOP.
- One final focused commit after reporting validation; no push, deployment or robot commands.

## Task 1: Direct read-only mapping health

Files: new `savo_mapping/include/savo_mapping/local_mapping_health.hpp`, `src/core/local_mapping_health.cpp`, `test/test_local_mapping_health.cpp`; extend `src/nodes/mapping_supervisor_node.cpp`; register library/tests in mapping CMake.

Interface: `/savo_mapping/local_health` (`std_msgs/String`, reliable volatile KeepLast(1)), schema version 1, producer `savo_mapping`, booleans `admission_ready`, `continuation_ready`, `semantic_ready`, and source-specific `reason`. Consumers must check actual monotonic receipt freshness (1.5s), validity and schema. This is evidence, never mission authority.

- [x] Write failing C++ tests using direct production payload fixtures: healthy sources admit in STOP; stale/error/invalid/time regression deny; environmental STOP denies admission but permits continuation; LOW denies new admission but permits continuation; required critical power denies; optional Edge absent permits.
- [x] Observe failures with native gtest before implementing.
- [x] Add pure parsers/evaluator and integrate direct subscriptions in existing read-only mapping supervisor. Reuse actual subsystem topics and values, not general Supervisor outputs.
- [x] Preserve scan/map/odom/TF readiness and observe SLAM lifecycle; consume Nav readiness infrastructure separately from NAV permission to avoid a cycle.
- [x] Run native tests, mapping contracts, lint/diff checks; report evidence for review.

Example behavior assertion:

```cpp
EXPECT_FALSE(decision.admission_ready);  // live environmental obstacle
EXPECT_TRUE(decision.continuation_ready);
```

## Task 2: Mission-bound local authority

Files: new mapping local-authority pure C++ library/test, `autonomous_mapping_orchestrator_node.cpp`, mission inputs/tests/config, `RunAutonomousMapping.action` comments if needed.

- [x] Reproduce current Supervisor requirement in a failing test.
- [x] Replace autonomous service acquisition/check/release with an in-process local lease; retain request identity and adopt a new local generation only for generation zero. Reject nonzero foreign/pre-acquired generations explicitly, not silently.
- [x] Validate current local-health evidence on admission and every execution evaluation. Revoke on invalid/stale evidence; do not reacquire automatically.
- [x] Preserve existing mission sequencing and control STOP acknowledgement/terminal cleanup. Rename internal Supervisor-specific authority fields to local authority.
- [x] Expose narrowly scoped child CHECK under `/savo_mapping/autonomous/authorize_phase` using typed context; never expose a second autonomous ACQUIRE surface.
- [x] Test identity mismatch, stale response/generation, no Supervisor process, startup STOP, admission NAV, environmental continuation, required-fault STOP and cancellation/completion STOP.

## Task 3: Child phase and caller integration

Files: Coverage operation orchestrator/core/config/tests; semantic registration/review and orchestrator phase bindings where required; bridge autonomous command submission/tests only if needed for action compatibility.

- [x] Add failing tests for Coverage/semantic execution under local mission identity without system Supervisor.
- [x] Explicitly select local parent authority for autonomous children; retain standalone system authority configuration. Local child permission checks must match mission actor/map/generation and operation.
- [x] Update autonomous callers to request generation-zero local acquisition, not obtain a system lease first. Preserve non-mapping bridge behavior.
- [x] Run phase behavior and standalone compatibility regressions.

## Task 4: Dedicated launch composition and documentation

Files: autonomous mapping Python/XML launch, config, deployment/launch contracts; mapping/bringup docs and Core validation instructions.

- [x] Add failing launch tests for `start_supervisor=false`, no automatic action, STOP startup and disabled optional Edge/head requirements.
- [x] Set dedicated autonomous launch default `start_supervisor=false`; leave normal/system bringup unchanged.
- [x] Pass local child authority options only within autonomous composition; retain Scan360 server availability without sending a goal.
- [x] Document action generation semantics, ownership and exact safe Core validation sequence.

## Task 5: Integration review and final validation

- [x] Review all changed files for authority gaps, stale evidence, actor/generation mismatches, cancellation races and inadvertent normal-mode changes.
- [x] Run hardware-free mapping, bringup, Nav, control, perception, localization, power and Supervisor compatibility tests. Compile native C++ where available; identify any ROS-time stubs explicitly.
- [x] Run ament-equivalent cpplint/uncrustify, flake8, Python compilation, YAML/XML validation and deployment contract validators.
- [x] Verify zero base/description/TF/hardware configuration diff and no unsupported Supervisor dependency in local autonomous execution.
- [x] Document exact available validation and remaining Pi/robot tests before committing.

```bash
git diff --check
git diff --stat
git status --short
```

Ruling: work in the current clean Mac checkout, per the user's explicit request; do not create another worktree or intermediate commits. Scratch native binaries and review artifacts stay outside the repository.

Implementation note: volatile latest-only local snapshots replace the initially proposed retained snapshots, so a new subscriber cannot accept an old retained ready state as fresh evidence. Direct input freshness remains steady-clock based. Validation and operator commands are in `docs/validation/mapping_local_authority.md`; real ROS node compilation/runtime and FastDDS checks remain explicitly pending Pi validation. The independent review agent stopped at its execution quota without a verdict; final code-path review and local validation were performed by the main agent.
