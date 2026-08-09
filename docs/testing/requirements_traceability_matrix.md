# Robot Savo Requirements Traceability Matrix

## Scope and method

These requirements are extracted from current production source, interfaces, launch/configuration, deployment and operator contracts. They are intentionally atomic. Source paths identify the implementation owner; documentation supplies the engineering contract but does not override source.

Evidence status uses [the evidence vocabulary](test_evidence_guidelines.md). `SOURCE_PASS` means the cited source/contract validator passed for the current checkout; it is not hardware evidence. `HISTORICAL_BASELINE` records earlier physical evidence whose current affected path still needs regression.

## Architecture, safety and motion

| Requirement ID | Requirement | Source/owner | Implementation | Verification | Evidence status |
| --- | --- | --- | --- | --- | --- |
| SYS-ARCH-001 | Production deployment shall contain exactly the current 20 ROS packages and role membership shall come from the Core/Edge role arrays. | Deployment | `deploy/{core,edge}/env_*.sh`; 20 `package.xml` files | BRG-001, package inventory audit | `SOURCE_PASS` |
| SYS-ARCH-002 | Core shall own movement, near-field safety, localization, mapping, navigation, locations, head and global supervision; Edge shall not directly own drivetrain execution. | Architecture/deployment | role arrays; package launch graphs | BRG-002, BRD-002, OBS-004 | `SOURCE_PASS` |
| SYS-ARCH-003 | `savo_msgs` shall provide the installed shared message/service/action definitions consumed by typed package APIs. | `savo_msgs` | `savo_ws/src/shared/savo_msgs/{msg,srv,action}`; CMake generation | interface build/inventory; affected producer/consumer tests | `NOT_RUN` |
| SYS-ARCH-004 | UI and observer shall be presentation-only and shall not acquire robot mutation authority. | `savo_ui`, `savo_observer` | UI production sources; observer sources/RViz; observer validator | UI-002, OBS-001/004 | `SOURCE_PASS` |
| SYS-SAF-001 | The final production motor path shall be control → perception gate → `/cmd_vel_safe` → base; routine procedures shall not bypass it. | Control/perception/base | control mux/shaper; perception gate; `base_driver_node` | CTL-005–007, PER-003/005, BAS-003/006 | `HISTORICAL_BASELINE` |
| SYS-SAF-002 | Required stale or invalid near-field safety evidence shall close motion rather than preserve a last-known safe state. | `savo_perception` | range health, safety stop and command-gate sources/config | PER-002–006/010 | `HISTORICAL_BASELINE` |
| SYS-SAF-003 | Production control shall initialize and fall back to STOP. | `savo_control`, bringup | control mode config; role launch defaults | CTL-002–004, BRG-003/007 | `SOURCE_PASS` |
| SYS-SAF-004 | Mission permission shall be distinct from subsystem execution; supervisor shall not write motors or plan/execute missions. | `savo_supervisor` | supervisor policy/node and typed services/cancel clients | SUP-001–006/009/010 | `NOT_RUN` |
| SYS-SAF-005 | A physical emergency stop and explicit return-to-service process shall control actuating tests. | Operations/hardware (external physical requirement) | emergency-stop procedures/checklists and installed robot hardware | SYS acceptance Gates C/G/P; failure/abort procedure | `BLOCKED` |
| SYS-MOT-001 | `savo_base` shall be the sole drivetrain hardware writer and consume only `/cmd_vel_safe`. | `savo_base` | production C++ base driver/config | BAS-001/003/005; source owner audit | `HISTORICAL_BASELINE` |
| SYS-MOT-002 | Base output shall zero on STOP, stale command/publisher loss, board failure and shutdown. | `savo_base` | driver watchdog/stop/error/shutdown paths | BAS-002–004/007/009 | `HISTORICAL_BASELINE` |
| SYS-MOT-003 | Control shall select only the fresh source allowed by current mode and shape it within configured limits. | `savo_control` | mode manager, mux and shaper | CTL-001–007/009/010 | `NOT_RUN` |
| SYS-MOT-004 | Recovery/rotation/approach commands shall use typed or approved command lanes and remain subject to safety/base gates. | `savo_control` | recovery, rotate action, approach and command topics | CTL-008–010; MAP-012; NAV-011/012 | `BLOCKED` |

## Bringup, perception and transforms

| Requirement ID | Requirement | Source/owner | Implementation | Verification | Evidence status |
| --- | --- | --- | --- | --- | --- |
| SYS-BRG-001 | Bringup shall reject invalid role, mode and profile values. | `savo_bringup` | `robot_bringup.launch.py`; bringup contract | BRG-001/002/004 | `SOURCE_PASS` |
| SYS-BRG-002 | Required component freshness shall determine Core and Edge readiness independently; Edge readiness shall not grant Core motion. | `savo_bringup` | readiness contract/node/config | BRG-004/006/009/010 | `NOT_RUN` |
| SYS-BRG-003 | Motion-capable profiles shall reject provisional geometry and optional voxel use shall require its independent validation flag. | `savo_bringup`, description/perception | bringup launch/profile gates; geometry validator | BRG-003/005; DSC-002/006; PER-007–010 | `SOURCE_PASS` |
| SYS-BRG-004 | Core-only, Edge-only and distributed safe-idle startup/shutdown shall avoid duplicate owners and preserve STOP. | Deployment/bringup | role launch graphs and deploy wrappers | BRG-007–010 | `BLOCKED` |
| SYS-PER-001 | Core shall own left/right ToF through the configured mux and the front ultrasonic sensor, with explicit identity and freshness. | `savo_perception` | sensor nodes and Core config | PER-001–004/010 | `BLOCKED` |
| SYS-PER-002 | The command safety gate shall never increase an upstream command and shall timeout to zero. | `savo_perception` | `cmd_vel_safety_gate` source/config | PER-002/003/005 | `HISTORICAL_BASELINE` |
| SYS-PER-003 | The Edge D435 obstacle cloud shall remain optional, independently filtered/validated, and not replace Core near-field safety. | `savo_perception` | obstacle cloud node/config and bringup validation flag | PER-007–010; RLS-006/007; NAV-014 | `BLOCKED` |
| SYS-TF-001 | `savo_description` shall own fixed robot transforms and shall not publish `map -> odom` or `odom -> base_footprint`. | `savo_description` | Xacro/URDF and description launch | DSC-003/007/010; TF audit | `NOT_RUN` |
| SYS-TF-002 | Localization EKF shall be the sole `odom -> base_footprint` owner; wheel odometry shall not duplicate it. | `savo_localization` | EKF/wheel odom configs | LOC-001/003/007/008 | `BLOCKED` |
| SYS-TF-003 | SLAM or AMCL/Nav2 shall own `map -> odom` according to mode, without simultaneous conflicting owners. | Mapping/navigation | mapping/Nav2 launch/config | MAP-003/008; NAV-003/008/009; TF audit | `BLOCKED` |
| SYS-TF-004 | Dynamic head transforms shall be owned by `savo_head` and fixed camera optical frames shall obey ROS optical convention. | `savo_head`, description | head TF node/config; description Xacro | HED-001/003/005/006; DSC-007/010 | `BLOCKED` |

## Mapping, data, navigation and locations

| Requirement ID | Requirement | Source/owner | Implementation | Verification | Evidence status |
| --- | --- | --- | --- | --- | --- |
| SYS-MAP-001 | Mapping save, artifact verification, quality evaluation, operator review and production release shall remain separate stages. | `savo_mapping` | session/save/quality/catalog/release sources/config | MAP-001/002/005/006/010/014 | `NOT_RUN` |
| SYS-MAP-002 | Autonomous mapping goal handoff shall bound response, execution and stale-feedback time and support cancellation. | `savo_mapping` | exploration/coverage handoff and orchestrator/sequencer | MAP-002/004/011 | `NOT_RUN` |
| SYS-MAP-003 | A production map release shall be immutable, digest-verified and activated only after required quality/review/context gates. | `savo_mapping` | production release/catalog implementation | MAP-005–007/014 | `BLOCKED` |
| SYS-MAP-004 | Semantic detections shall create reviewable evidence/candidates and shall not approve operator-controlled locations automatically. | Mapping/head/locations | tag action, review gateway, location candidate services | HED-006; MAP-013; LCT-004/005/010 | `BLOCKED` |
| SYS-DAT-001 | Core production map, location and supervisor state shall use provisioned `/var/lib/robot_savo` paths rather than `/tmp`. | Deployment and state owners | storage preparation; launch/config defaults | BRG-001; LCT-001/003; MAP-001/005; SUP-001/003 | `SOURCE_PASS` |
| SYS-DAT-002 | Location and supervisor persistent writes shall be atomic/fail-closed and corrupt/newer state shall not be trusted. | `savo_locations`, `savo_supervisor` | SQLite/state-store implementations | LCT-002/007/008; SUP-007/008 | `NOT_RUN` |
| SYS-DAT-003 | Backup/restore/rollback shall preserve verified identities/digests and be tested on isolated/copied data. | Deployment/operations | backup/restore scripts and release rollback services | LCT-009; MAP-007/014; deployment backup test | `NOT_RUN` |
| SYS-DAT-004 | Runtime sockets under `/run` shall have explicit owner/group/mode and shall not become persistent authority. | Bridge/speech/deployment | bridge config/preparation; SavoMind runtime config; systemd/tmpfiles | BRD-001/004/008; SPH-001/004/009 | `NOT_RUN` |
| SYS-NAV-001 | Production saved-map navigation shall require verified active map/geometry/location context before lifecycle/admission. | `savo_nav` | active map context/readiness/gateway and production launch | NAV-001–004/007/009 | `BLOCKED` |
| SYS-NAV-002 | Public goal admission shall isolate internal Nav2 actions and reject stale, unsafe, unauthorized, invalid or mismatched goals. | `savo_nav` | goal gateway/admission/readiness sources | NAV-002/004/006/010/011 | `NOT_RUN` |
| SYS-NAV-003 | Navigation commands shall route through the nav lane, control, perception and base, and cancellation/revocation shall produce a bounded terminal state. | Nav/control/perception/base/supervisor | gateway/backend, `/cmd_vel_nav`, mux/gate/watchdog | NAV-004/006/010–012; SUP-010 | `BLOCKED` |
| SYS-NAV-004 | LiDAR-only shall be the validated baseline; the D435 voxel profile shall stay disabled until separate cloud/Nav2 physical validation passes. | Nav/perception/bringup | Nav2 profiles/readiness and D435 gate | PER-007–010; NAV-001/009/014; BRG-003 | `SOURCE_PASS` |
| SYS-LOC-001 | Location candidates shall require explicit authorized operator approval/rejection; SavoMind shall not receive approval authority. | `savo_locations`, supervisor/bridge | candidate services/policy; bridge deny surface | LCT-004/005/010; BRD-002 | `NOT_RUN` |
| SYS-LOC-002 | Only approved, enabled, unambiguous locations matching active map/release context shall resolve for navigation. | `savo_locations`, nav | resolve service and named-navigation gateway | LCT-004–006/010; NAV-005/013 | `BLOCKED` |
| SYS-LOC-003 | Location releases shall be digest-associated with the matching map/revision and current geometry context as required by the release contract. | Locations/mapping | release prepare/verify/commit/rollback implementation | LCT-007–010; MAP-014; NAV-007 | `BLOCKED` |

## Edge interaction, power, network, deployment and operations

| Requirement ID | Requirement | Source/owner | Implementation | Verification | Evidence status |
| --- | --- | --- | --- | --- | --- |
| SYS-SPH-001 | `savo_speech` shall own physical capture/playback and bounded transport, while SavoMind owns STT/LLM/TTS inference. | Speech/SavoMind boundary | speech runtime/config/protocol-v2 | SPH-001–006/010 | `SOURCE_PASS` |
| SYS-SPH-002 | Speech responses shall match request/session identity and valid bounded WAV format before playback. | `savo_speech` | protocol/WAV/session/playback implementation | SPH-002–004/006 | `NOT_RUN` |
| SYS-SPH-003 | Playback cancellation and microphone gating shall prevent unbounded feedback/replay and recover honestly after device/socket loss. | `savo_speech` | playback/capture runtime and health | SPH-002/008/009 | `BLOCKED` |
| SYS-UI-001 | UI shall display required robot states with explicit freshness/stale/offline presentation and safety priority. | `savo_ui` | UI render/freshness/state sources/config | UI-001/003–007 | `BLOCKED` |
| SYS-UI-002 | Observer shall reject motion/goal/initial-pose/service/action authority and unsafe RViz tools. | `savo_observer` | observer sources/RViz and validator | OBS-001/002/004 | `SOURCE_PASS` |
| SYS-PWR-001 | Power monitoring shall distinguish raw validity and calibrated/un-calibrated percentage; stale/invalid sources shall not appear healthy. | `savo_power` | role drivers, aggregator/health config | PWR-001–007 | `BLOCKED` |
| SYS-PWR-002 | Low-power handling shall publish a request; `savo_power` and supervisor shall not directly execute OS shutdown. | Power/supervisor/deployment | shutdown-request publishers and privileged consumer boundary | PWR-003/006/007; SUP-006/009 | `NOT_RUN` |
| SYS-NET-001 | Core, Edge and observer shall use the approved ROS domain/middleware/network and fresh synchronized time for distributed data. | Deployment/network | env files, netplan, Chrony/RMW/domain setup | BRG-009/010; OBS-005–007; timestamp integration | `BLOCKED` |
| SYS-NET-002 | Edge local sockets shall validate their parent/permissions and peer identity where configured, with bounded framing/timeouts. | Bridge/speech/deployment | bridge/speech socket implementations/config | BRD-001–006/008; SPH-001–004/009 | `NOT_RUN` |
| SYS-DEP-001 | Role build/dependency scripts shall verify complete approved package membership and shall not silently allow missing production packages. | Deployment | role build/install scripts and arrays | BRG-001; clean role builds/tests | `NOT_RUN` |
| SYS-DEP-002 | Production service ownership shall avoid duplicate generic/role/component owners and preserve safe startup/shutdown/restart defaults. | Deployment/systemd/bringup | systemd templates/renderer and role launch flags | BRG-005/007–010; systemd audit | `BLOCKED` |
| SYS-DEP-003 | Release/update/rollback shall exclude generated build artifacts and preserve recoverable previous installation/state. | Deployment/operations | update scripts, `.gitignore`, backup/rollback procedures | artifact/git hygiene and isolated rollback tests | `NOT_RUN` |
| SYS-OPS-001 | Operators shall verify physical inspection, STOP, readiness, geometry and required lower gates before actuating tests. | Operations/testing | pre-operation, startup, manual/mapping/nav and acceptance procedures | SYS acceptance Gates A–M; relevant component prerequisites | `BLOCKED` |
| SYS-OPS-002 | Abort handling shall STOP, isolate, preserve evidence, diagnose, retest prerequisites and require explicit reauthorization. | Operations/testing | failure/abort criteria and incident procedures | observed abort drill/acceptance Gate P | `BLOCKED` |
| SYS-OPS-003 | Production return to service shall be an explicit reviewed decision with evidence, operating envelope and closed blockers/deviations. | Operations/acceptance owner | acceptance checklist and incident/recovery procedures | SYS acceptance final decision | `BLOCKED` |

## Coverage summary

| Metric | Count |
| --- | ---: |
| Total requirements | 54 |
| Requirements with implementation trace | 54 |
| Requirements with test trace | 54 |
| Requirements with current source or historical evidence | 15 |
| Requirements `BLOCKED` | 22 |
| Requirements `NOT_RUN` | 17 |
| Requirements with no implementation owner | 0 |
| Production requirements with no verification method | 0 |

`BLOCKED` and `NOT_RUN` entries are open verification work, not missing traceability. The current evidence count includes `SOURCE_PASS` and `HISTORICAL_BASELINE`; neither implies target/hardware/acceptance validation.
