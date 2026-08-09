# Hardware Revision History

This repository does not currently contain enough as-built evidence to assign a verified robot hardware revision. The entries below distinguish software geometry profiles and historical claims from a physical revision.

| Identifier/date | Classification | Evidence and disposition |
| --- | --- | --- |
| Earlier Robot Savo baseline (date not reconstructed here) | Historical hardware exercise | Prior documentation reports physical operation; does not validate current source/configuration |
| `robot_savo_core_v1` | Software geometry profile | Current selected profile; `measurement_state: provisional`; not an as-built revision |
| 2026-08-09 Phase 4 audit | Documentation baseline | Source-derived inventory created; physical measurements and serials remain pending |

## Required revision record

Create the first verified hardware revision only after physical inspection. Record:

- revision ID, build/commission date, custodian, and robot asset/serial;
- BOM manufacturer parts, quantities, board revisions, serials, and substitutions;
- mechanical drawings/bracket/deck revisions and fastener/standoff details;
- cable/harness revision and power-tree/fuse/connector diagram;
- measured/locked geometry profile and calibration evidence;
- firmware/EEPROM settings and source/deployment release compatibility;
- safety-impact assessment, validation plan/result, photos, and approver.

Any change to wheels, encoder gearing, motors, power path, sensor type/mount, compute board, PCA allocation, head mechanics, wiring, or mass distribution requires a new revision or controlled deviation. It also reopens the affected calibration and real-robot tests. Documentation-only corrections may amend a record without claiming a physical rebuild, but must retain the amendment history.
