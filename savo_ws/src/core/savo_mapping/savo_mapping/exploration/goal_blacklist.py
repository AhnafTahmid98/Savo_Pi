#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Exploration goal blacklist helpers for Robot Savo mapping."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

from savo_mapping.exploration.goal_selector import GoalBlacklistEntry
from savo_mapping.models.exploration_status import ExplorationGoal


@dataclass(frozen=True)
class BlacklistRecord:
    x: float
    y: float
    radius_m: float = 0.60
    reason: str = "failed_goal"
    source: str = "exploration"
    created_at_s: float = field(default_factory=time.time)
    expires_at_s: Optional[float] = None
    hit_count: int = 0

    @property
    def expired(self) -> bool:
        return self.expires_at_s is not None and time.time() >= self.expires_at_s

    def contains(self, x: float, y: float) -> bool:
        return math.hypot(float(x) - self.x, float(y) - self.y) <= self.radius_m

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(float(x) - self.x, float(y) - self.y)

    def with_hit(self) -> "BlacklistRecord":
        return BlacklistRecord(
            x=self.x,
            y=self.y,
            radius_m=self.radius_m,
            reason=self.reason,
            source=self.source,
            created_at_s=self.created_at_s,
            expires_at_s=self.expires_at_s,
            hit_count=self.hit_count + 1,
        )

    def to_goal_selector_entry(self) -> GoalBlacklistEntry:
        return GoalBlacklistEntry(
            x=self.x,
            y=self.y,
            radius_m=self.radius_m,
            reason=self.reason,
        )

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "radius_m": self.radius_m,
            "reason": self.reason,
            "source": self.source,
            "created_at_s": self.created_at_s,
            "expires_at_s": self.expires_at_s,
            "expired": self.expired,
            "hit_count": self.hit_count,
        }


@dataclass(frozen=True)
class BlacklistQueryResult:
    blocked: bool
    record: Optional[BlacklistRecord] = None
    distance_m: Optional[float] = None
    message: str = "Goal not blacklisted."

    def to_dict(self) -> dict:
        return {
            "blocked": self.blocked,
            "record": self.record.to_dict() if self.record is not None else None,
            "distance_m": self.distance_m,
            "message": self.message,
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


@dataclass
class GoalBlacklist:
    default_radius_m: float = 0.60
    default_ttl_s: Optional[float] = 300.0
    max_records: int = 100
    records: list[BlacklistRecord] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.default_radius_m <= 0.0:
            raise ValueError("default_radius_m must be positive.")

        if self.default_ttl_s is not None and self.default_ttl_s <= 0.0:
            raise ValueError("default_ttl_s must be positive or None.")

        if self.max_records <= 0:
            raise ValueError("max_records must be positive.")

    @property
    def active_records(self) -> tuple[BlacklistRecord, ...]:
        self.prune_expired()
        return tuple(self.records)

    @property
    def count(self) -> int:
        return len(self.active_records)

    def add(
        self,
        x: float,
        y: float,
        *,
        radius_m: Optional[float] = None,
        reason: str = "failed_goal",
        source: str = "exploration",
        ttl_s: Optional[float] = None,
    ) -> BlacklistRecord:
        ttl = self.default_ttl_s if ttl_s is None else ttl_s
        expires_at_s = None if ttl is None else time.time() + float(ttl)

        record = BlacklistRecord(
            x=float(x),
            y=float(y),
            radius_m=float(radius_m or self.default_radius_m),
            reason=str(reason),
            source=str(source),
            expires_at_s=expires_at_s,
        )

        self.records.append(record)
        self.prune_expired()
        self._trim_old_records()

        return record

    def add_goal(
        self,
        goal: ExplorationGoal,
        *,
        radius_m: Optional[float] = None,
        reason: str = "failed_goal",
        source: str = "exploration",
        ttl_s: Optional[float] = None,
    ) -> BlacklistRecord:
        return self.add(
            goal.x,
            goal.y,
            radius_m=radius_m,
            reason=reason,
            source=source,
            ttl_s=ttl_s,
        )

    def check(self, x: float, y: float) -> BlacklistQueryResult:
        self.prune_expired()

        for index, record in enumerate(self.records):
            distance = record.distance_to(x, y)

            if not record.contains(x, y):
                continue

            updated = record.with_hit()
            self.records[index] = updated

            return BlacklistQueryResult(
                blocked=True,
                record=updated,
                distance_m=distance,
                message=f"Goal blocked by blacklist: {updated.reason}.",
            )

        return BlacklistQueryResult(blocked=False)

    def check_goal(self, goal: ExplorationGoal) -> BlacklistQueryResult:
        return self.check(goal.x, goal.y)

    def clear(self) -> None:
        self.records.clear()

    def prune_expired(self) -> int:
        before = len(self.records)
        self.records = [record for record in self.records if not record.expired]
        return before - len(self.records)

    def to_goal_selector_entries(self) -> tuple[GoalBlacklistEntry, ...]:
        return tuple(
            record.to_goal_selector_entry()
            for record in self.active_records
        )

    def to_dict(self) -> dict:
        return {
            "default_radius_m": self.default_radius_m,
            "default_ttl_s": self.default_ttl_s,
            "max_records": self.max_records,
            "count": self.count,
            "records": [record.to_dict() for record in self.active_records],
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)

    @classmethod
    def from_records(
        cls,
        records: Sequence[BlacklistRecord],
        *,
        default_radius_m: float = 0.60,
        default_ttl_s: Optional[float] = 300.0,
        max_records: int = 100,
    ) -> "GoalBlacklist":
        blacklist = cls(
            default_radius_m=default_radius_m,
            default_ttl_s=default_ttl_s,
            max_records=max_records,
        )
        blacklist.records = list(records)
        blacklist.prune_expired()
        blacklist._trim_old_records()
        return blacklist

    def _trim_old_records(self) -> None:
        if len(self.records) <= self.max_records:
            return

        self.records = self.records[-self.max_records:]


def blacklist_record_from_dict(data: dict) -> BlacklistRecord:
    return BlacklistRecord(
        x=float(data["x"]),
        y=float(data["y"]),
        radius_m=float(data.get("radius_m", 0.60)),
        reason=str(data.get("reason", "failed_goal")),
        source=str(data.get("source", "exploration")),
        created_at_s=float(data.get("created_at_s", time.time())),
        expires_at_s=(
            None
            if data.get("expires_at_s") is None
            else float(data["expires_at_s"])
        ),
        hit_count=int(data.get("hit_count", 0)),
    )


def blacklist_from_dict(data: dict) -> GoalBlacklist:
    records = [
        blacklist_record_from_dict(item)
        for item in data.get("records", ())
    ]

    return GoalBlacklist.from_records(
        records,
        default_radius_m=float(data.get("default_radius_m", 0.60)),
        default_ttl_s=data.get("default_ttl_s", 300.0),
        max_records=int(data.get("max_records", 100)),
    )


def main() -> None:
    blacklist = GoalBlacklist(default_ttl_s=None)

    record = blacklist.add(
        2.0,
        3.0,
        reason="nav2_failed",
    )

    query = blacklist.check(2.1, 3.1)

    print(record.to_dict())
    print(query.to_json(indent=2))
    print(blacklist.to_json(indent=2))


if __name__ == "__main__":
    main()


ExplorationBlacklist = GoalBlacklist


__all__ = [
    "BlacklistQueryResult",
    "BlacklistRecord",
    "GoalBlacklist",
    "ExplorationBlacklist",
    "blacklist_from_dict",
    "blacklist_record_from_dict",
    "main",
]
