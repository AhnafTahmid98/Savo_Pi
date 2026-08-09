# Incident Response

## Scope and severity

| Category | Technical meaning | Release authority |
| --- | --- | --- |
| Operational anomaly | No hazardous motion/contact; bounded loss of optional function | Maintainer after documented correction/checks |
| Safety-relevant incident | Unexpected motion, near miss, safety/localization/power failure during motion | Safety review required |
| Critical incident | Person/property contact, failure to stop, emergency-control failure, authority bypass, fire/battery hazard | Isolate; formal engineering and safety approval |

These are engineering categories, not legal classifications.

## Incident lifecycle

1. **Make safe.** Use physical isolation and the
   [emergency procedure](emergency_stop_and_recovery.md).
2. **Protect people/property.** Provide assistance, restrict access, and follow
   site emergency policy.
3. **Preserve evidence.** Do not clear logs, power-cycle repeatedly, edit state,
   or disturb hardware unless needed for safety.
4. **Isolate.** Tag the robot out of service; protect batteries and affected
   network/accounts as applicable.
5. **Record identity.** Capture time/timezone, operators, hardware revision,
   commits/installs, SavoMind version, geometry digest, mode/profile, active
   map/location release, authority, safety, localization, power, and network.
6. **Collect evidence.** Use [log collection](log_collection.md), photographs,
   video, artifacts, and an approved ROS bag when safe.
7. **Investigate.** Reconstruct command source, admission, control/safety/base
   path, TF/localization, service restarts, power, and physical response.
8. **Correct.** Change source/config/hardware/state only through review; retain
   old evidence and record every change.
9. **Regression test.** Begin isolated/wheels-raised, then component,
   integration, and guarded site tests selected from the failure mode.
10. **Review and release.** A named reviewer explicitly approves or rejects
    return to service. Successful startup alone never closes an incident.

## Mandatory escalation

Safety review is required for collision/contact, near miss, uncommanded motion,
failed/late stop, safety sensor bypass/false clear, localization/TF failure
during motion, unauthorized command/authority, repeated hazardous recovery,
battery/fire event, or corrupted state admitted to navigation.

Security events also require preservation of socket permissions, peer identity,
bridge rejection/result, exposed credentials scope, and access logs. Rotate
credentials through the approved process; never attach secrets to the report.

Use the [incident report template](incident_report_template.md) and link all
corrective changes, test evidence, reviewers, residual risks, and restrictions.
