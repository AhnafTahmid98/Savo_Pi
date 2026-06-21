# -*- coding: utf-8 -*-

"""Topic freshness helpers for diagnostics and dashboards."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from savo_control.ros import qos_name_for_topic

from .health import HealthCheck, make_health_report


@dataclass(frozen=True)
class TopicProbe:
    name: str
    topic: str
    required: bool = True
    timeout_s: float = 0.50
    last_seen_s: float | None = None
    count: int = 0
    last_value: object = ""
    qos_name: str = ""

    @classmethod
    def create(
        cls,
        *,
        name: str,
        topic: str,
        required: bool = True,
        timeout_s: float = 0.50,
        qos_name: str = "",
    ) -> "TopicProbe":
        return cls(
            name=name,
            topic=topic,
            required=required,
            timeout_s=max(0.0, float(timeout_s)),
            qos_name=qos_name or qos_name_for_topic(topic),
        )

    def observed(
        self,
        *,
        now_s: float,
        value: object = "",
    ) -> "TopicProbe":
        return TopicProbe(
            name=self.name,
            topic=self.topic,
            required=self.required,
            timeout_s=self.timeout_s,
            last_seen_s=float(now_s),
            count=self.count + 1,
            last_value=value,
            qos_name=self.qos_name,
        )

    def age_s(self, *, now_s: float) -> float | None:
        if self.last_seen_s is None:
            return None

        return max(0.0, float(now_s) - self.last_seen_s)

    def fresh(self, *, now_s: float) -> bool:
        age = self.age_s(now_s=now_s)

        if age is None:
            return False

        return age <= self.timeout_s

    def stale(self, *, now_s: float) -> bool:
        return not self.fresh(now_s=now_s)

    def health_check(self, *, now_s: float) -> HealthCheck:
        age = self.age_s(now_s=now_s)

        if age is None:
            return HealthCheck(
                name=self.name,
                ok=not self.required,
                stale=self.required,
                value="never",
                note=self.topic,
            )

        is_stale = age > self.timeout_s

        return HealthCheck(
            name=self.name,
            ok=not is_stale,
            stale=is_stale,
            value=f"{age:.3f}s",
            note=self.topic,
        )

    def to_dict(self, *, now_s: float | None = None) -> dict:
        age = None if now_s is None else self.age_s(now_s=now_s)

        return {
            "name": self.name,
            "topic": self.topic,
            "required": self.required,
            "timeout_s": self.timeout_s,
            "last_seen_s": self.last_seen_s,
            "age_s": age,
            "count": self.count,
            "last_value": self.last_value,
            "qos_name": self.qos_name,
        }

    def status_text(self, *, now_s: float) -> str:
        age = self.age_s(now_s=now_s)
        age_text = "never" if age is None else f"{age:.3f}s"
        state = "fresh" if self.fresh(now_s=now_s) else "stale"

        return (
            f"{self.name}: {state}; topic={self.topic}; "
            f"age={age_text}; count={self.count}; qos={self.qos_name}"
        )


@dataclass(frozen=True)
class TopicProbeSet:
    name: str
    probes: tuple[TopicProbe, ...]

    def update(
        self,
        topic: str,
        *,
        now_s: float,
        value: object = "",
    ) -> "TopicProbeSet":
        updated = []

        for probe in self.probes:
            if probe.topic == topic:
                updated.append(probe.observed(now_s=now_s, value=value))
            else:
                updated.append(probe)

        return TopicProbeSet(name=self.name, probes=tuple(updated))

    def health_checks(self, *, now_s: float) -> list[HealthCheck]:
        return [probe.health_check(now_s=now_s) for probe in self.probes]

    def health_report(self, *, now_s: float):
        return make_health_report(self.name, self.health_checks(now_s=now_s))

    def missing_required(self, *, now_s: float) -> list[TopicProbe]:
        return [
            probe
            for probe in self.probes
            if probe.required and probe.stale(now_s=now_s)
        ]

    def to_dict(self, *, now_s: float | None = None) -> dict:
        report = None if now_s is None else self.health_report(now_s=now_s)

        return {
            "name": self.name,
            "overall": None if report is None else report.overall,
            "probes": [probe.to_dict(now_s=now_s) for probe in self.probes],
        }

    def status_text(self, *, now_s: float) -> str:
        lines = [self.health_report(now_s=now_s).status_text()]
        lines.extend(probe.status_text(now_s=now_s) for probe in self.probes)
        return "\n".join(lines)


def make_topic_probe_set(name: str, probes: Iterable[TopicProbe]) -> TopicProbeSet:
    return TopicProbeSet(name=name, probes=tuple(probes))


def control_topic_probe_set() -> TopicProbeSet:
    return make_topic_probe_set(
        "savo_control_topics",
        [
            TopicProbe.create(name="cmd_vel", topic="/cmd_vel", timeout_s=0.30),
            TopicProbe.create(name="cmd_vel_safe", topic="/cmd_vel_safe", timeout_s=0.30),
            TopicProbe.create(name="mode_state", topic="/savo_control/mode_state", timeout_s=0.80),
            TopicProbe.create(name="safety_stop", topic="/safety/stop", timeout_s=0.50),
            TopicProbe.create(name="slowdown_factor", topic="/safety/slowdown_factor", timeout_s=0.50),
            TopicProbe.create(name="odometry", topic="/odometry/filtered", timeout_s=0.50),
            TopicProbe.create(name="recovery_status", topic="/savo_control/recovery_status", required=False, timeout_s=1.00),
        ],
    )


__all__ = [
    "TopicProbe",
    "TopicProbeSet",
    "control_topic_probe_set",
    "make_topic_probe_set",
]
