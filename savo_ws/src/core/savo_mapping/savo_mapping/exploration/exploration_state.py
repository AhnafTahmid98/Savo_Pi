#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Exploration state models for Robot Savo mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, replace
from typing import Any, Optional, Sequence

from savo_mapping.models.exploration_status import ExplorationGoal


STATE_IDLE = "idle"
STATE_RUNNING = "running"
STATE_PAUSED = "paused"
STATE_WAITING = "waiting"
STATE_COMPLETE = "complete"
STATE_STOPPED = "stopped"
STATE_ERROR = "error"

EVENT_STARTED = "started"
EVENT_PAUSED = "paused"
EVENT_RESUMED = "resumed"
EVENT_WAITING = "waiting"
EVENT_GOAL_SELECTED = "goal_selected"
EVENT_GOAL_REACHED = "goal_reached"
EVENT_GOAL_FAILED = "goal_failed"
EVENT_GOAL_ATTEMPT_FINISHED = "goal_attempt_finished"
EVENT_COMPLETED = "completed"
EVENT_STOPPED = "stopped"
EVENT_ERROR = "error"


@dataclass(frozen=True)
class ExplorationEvent:
    event: str
    message: str = ""
    timestamp_s: float = field(default_factory=time.time)
    data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "event": self.event,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "data": dict(self.data),
        }


@dataclass(frozen=True)
class ExplorationGoalAttempt:
    goal: ExplorationGoal
    status: str = "active"
    reason: str = ""
    started_at_s: float = field(default_factory=time.time)
    finished_at_s: Optional[float] = None

    @property
    def active(self) -> bool:
        return self.status == "active"

    @property
    def duration_s(self) -> Optional[float]:
        if self.finished_at_s is None:
            return None

        return max(0.0, self.finished_at_s - self.started_at_s)

    def finish(self, status: str, reason: str = "") -> "ExplorationGoalAttempt":
        return replace(
            self,
            status=str(status),
            reason=str(reason),
            finished_at_s=time.time(),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "goal": self.goal.to_dict(),
            "status": self.status,
            "reason": self.reason,
            "started_at_s": self.started_at_s,
            "finished_at_s": self.finished_at_s,
            "duration_s": self.duration_s,
            "active": self.active,
        }


@dataclass(frozen=True)
class ExplorationCounters:
    selected_goals: int = 0
    reached_goals: int = 0
    failed_goals: int = 0
    no_candidate_cycles: int = 0
    pause_count: int = 0
    error_count: int = 0

    def with_selected_goal(self) -> "ExplorationCounters":
        return replace(self, selected_goals=self.selected_goals + 1)

    def with_reached_goal(self) -> "ExplorationCounters":
        return replace(self, reached_goals=self.reached_goals + 1)

    def with_failed_goal(self) -> "ExplorationCounters":
        return replace(self, failed_goals=self.failed_goals + 1)

    def with_no_candidate_cycle(self) -> "ExplorationCounters":
        return replace(
            self,
            no_candidate_cycles=self.no_candidate_cycles + 1,
        )

    def reset_no_candidate_cycles(self) -> "ExplorationCounters":
        return replace(self, no_candidate_cycles=0)

    def with_pause(self) -> "ExplorationCounters":
        return replace(self, pause_count=self.pause_count + 1)

    def with_error(self) -> "ExplorationCounters":
        return replace(self, error_count=self.error_count + 1)

    def to_dict(self) -> dict[str, int]:
        return {
            "selected_goals": self.selected_goals,
            "reached_goals": self.reached_goals,
            "failed_goals": self.failed_goals,
            "no_candidate_cycles": self.no_candidate_cycles,
            "pause_count": self.pause_count,
            "error_count": self.error_count,
        }


@dataclass(frozen=True)
class ExplorationState:
    state: str = STATE_IDLE
    active_goal: Optional[ExplorationGoal] = None
    active_attempt: Optional[ExplorationGoalAttempt] = None
    last_goal: Optional[ExplorationGoal] = None
    pause_reason: str = ""
    stop_reason: str = ""
    error_message: str = ""
    counters: ExplorationCounters = field(default_factory=ExplorationCounters)
    events: tuple[ExplorationEvent, ...] = ()
    started_at_s: Optional[float] = None
    updated_at_s: float = field(default_factory=time.time)
    max_events: int = 50

    @property
    def active(self) -> bool:
        return self.state == STATE_RUNNING

    @property
    def paused(self) -> bool:
        return self.state == STATE_PAUSED

    @property
    def complete(self) -> bool:
        return self.state == STATE_COMPLETE

    @property
    def stopped(self) -> bool:
        return self.state == STATE_STOPPED

    @property
    def has_goal(self) -> bool:
        return self.active_goal is not None

    def start(self, message: str = "Exploration started.") -> "ExplorationState":
        return self._replace_with_event(
            state=STATE_RUNNING,
            pause_reason="",
            stop_reason="",
            error_message="",
            started_at_s=self.started_at_s or time.time(),
            event=ExplorationEvent(EVENT_STARTED, message),
        )

    def pause(self, reason: str) -> "ExplorationState":
        return self._replace_with_event(
            state=STATE_PAUSED,
            pause_reason=str(reason),
            counters=self.counters.with_pause(),
            event=ExplorationEvent(EVENT_PAUSED, str(reason)),
        )

    def resume(self, message: str = "Exploration resumed.") -> "ExplorationState":
        return self._replace_with_event(
            state=STATE_RUNNING,
            pause_reason="",
            event=ExplorationEvent(EVENT_RESUMED, message),
        )

    def wait(self, message: str = "Waiting for exploration input.") -> "ExplorationState":
        return self._replace_with_event(
            state=STATE_WAITING,
            event=ExplorationEvent(EVENT_WAITING, message),
        )

    def select_goal(
        self,
        goal: ExplorationGoal,
        *,
        reason: str = "frontier_selected",
    ) -> "ExplorationState":
        attempt = ExplorationGoalAttempt(
            goal=goal,
            status="active",
            reason=reason,
        )

        return self._replace_with_event(
            state=STATE_RUNNING,
            active_goal=goal,
            active_attempt=attempt,
            last_goal=goal,
            pause_reason="",
            counters=self.counters.with_selected_goal().reset_no_candidate_cycles(),
            event=ExplorationEvent(
                EVENT_GOAL_SELECTED,
                reason,
                data={"goal": goal.to_dict()},
            ),
        )

    def mark_goal_reached(
        self,
        reason: str = "goal_reached",
    ) -> "ExplorationState":
        finished_attempt = (
            self.active_attempt.finish("reached", reason)
            if self.active_attempt is not None
            else None
        )
        goal = self.active_goal

        return self._replace_with_event(
            state=STATE_RUNNING,
            active_goal=None,
            active_attempt=None,
            last_goal=goal or self.last_goal,
            counters=self.counters.with_reached_goal(),
            event=ExplorationEvent(
                EVENT_GOAL_REACHED,
                reason,
                data={"goal": goal.to_dict() if goal is not None else None},
            ),
            extra_event=_attempt_event(finished_attempt),
        )

    def mark_goal_failed(
        self,
        reason: str = "goal_failed",
    ) -> "ExplorationState":
        finished_attempt = (
            self.active_attempt.finish("failed", reason)
            if self.active_attempt is not None
            else None
        )
        goal = self.active_goal

        return self._replace_with_event(
            state=STATE_RUNNING,
            active_goal=None,
            active_attempt=None,
            last_goal=goal or self.last_goal,
            counters=self.counters.with_failed_goal(),
            event=ExplorationEvent(
                EVENT_GOAL_FAILED,
                reason,
                data={"goal": goal.to_dict() if goal is not None else None},
            ),
            extra_event=_attempt_event(finished_attempt),
        )

    def no_candidate_cycle(
        self,
        message: str = "No exploration candidate found.",
    ) -> "ExplorationState":
        return self._replace_with_event(
            state=STATE_WAITING,
            counters=self.counters.with_no_candidate_cycle(),
            event=ExplorationEvent(EVENT_WAITING, message),
        )

    def complete_run(
        self,
        message: str = "Exploration complete.",
    ) -> "ExplorationState":
        return self._replace_with_event(
            state=STATE_COMPLETE,
            active_goal=None,
            active_attempt=None,
            event=ExplorationEvent(EVENT_COMPLETED, message),
        )

    def stop(self, reason: str = "stopped") -> "ExplorationState":
        return self._replace_with_event(
            state=STATE_STOPPED,
            active_goal=None,
            active_attempt=None,
            stop_reason=str(reason),
            event=ExplorationEvent(EVENT_STOPPED, str(reason)),
        )

    def error(self, message: str) -> "ExplorationState":
        return self._replace_with_event(
            state=STATE_ERROR,
            error_message=str(message),
            counters=self.counters.with_error(),
            event=ExplorationEvent(EVENT_ERROR, str(message)),
        )

    def reset(self, *, keep_events: bool = False) -> "ExplorationState":
        return ExplorationState(
            events=self.events if keep_events else (),
            max_events=self.max_events,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "active": self.active,
            "paused": self.paused,
            "complete": self.complete,
            "stopped": self.stopped,
            "has_goal": self.has_goal,
            "active_goal": _goal_to_dict(self.active_goal),
            "active_attempt": (
                self.active_attempt.to_dict()
                if self.active_attempt is not None
                else None
            ),
            "last_goal": _goal_to_dict(self.last_goal),
            "pause_reason": self.pause_reason,
            "stop_reason": self.stop_reason,
            "error_message": self.error_message,
            "counters": self.counters.to_dict(),
            "events": [event.to_dict() for event in self.events],
            "started_at_s": self.started_at_s,
            "updated_at_s": self.updated_at_s,
            "max_events": self.max_events,
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)

    def _replace_with_event(
        self,
        *,
        event: ExplorationEvent,
        extra_event: Optional[ExplorationEvent] = None,
        **updates: Any,
    ) -> "ExplorationState":
        events = self._append_event(event)

        if extra_event is not None:
            events = self._append_event(extra_event, events=events)

        return replace(
            self,
            events=events,
            updated_at_s=time.time(),
            **updates,
        )

    def _append_event(
        self,
        event: ExplorationEvent,
        *,
        events: Optional[Sequence[ExplorationEvent]] = None,
    ) -> tuple[ExplorationEvent, ...]:
        base = tuple(self.events if events is None else events)
        updated = base + (event,)

        if len(updated) > self.max_events:
            updated = updated[-self.max_events:]

        return updated


def make_initial_exploration_state(
    *,
    max_events: int = 50,
) -> ExplorationState:
    return ExplorationState(max_events=max_events)


def exploration_state_from_dict(data: dict[str, Any]) -> ExplorationState:
    return ExplorationState(
        state=str(data.get("state", STATE_IDLE)),
        active_goal=_goal_from_dict(data.get("active_goal")),
        active_attempt=_attempt_from_dict(data.get("active_attempt")),
        last_goal=_goal_from_dict(data.get("last_goal")),
        pause_reason=str(data.get("pause_reason", "")),
        stop_reason=str(data.get("stop_reason", "")),
        error_message=str(data.get("error_message", "")),
        counters=_counters_from_dict(data.get("counters", {})),
        events=tuple(_event_from_dict(item) for item in data.get("events", ())),
        started_at_s=data.get("started_at_s"),
        updated_at_s=float(data.get("updated_at_s", time.time())),
        max_events=int(data.get("max_events", 50)),
    )


def _goal_to_dict(goal: Optional[ExplorationGoal]) -> Optional[dict[str, Any]]:
    if goal is None:
        return None

    return goal.to_dict()


def _goal_from_dict(data: Any) -> Optional[ExplorationGoal]:
    if data is None:
        return None

    if not isinstance(data, dict):
        raise ValueError("ExplorationGoal data must be a dictionary.")

    return ExplorationGoal(
        x=float(data.get("x", 0.0)),
        y=float(data.get("y", 0.0)),
        yaw=float(data.get("yaw", 0.0)),
        frame_id=str(data.get("frame_id", "map")),
        score=float(data.get("score", 0.0)),
        reason=str(data.get("reason", "")),
    )


def _attempt_from_dict(data: Any) -> Optional[ExplorationGoalAttempt]:
    if data is None:
        return None

    if not isinstance(data, dict):
        raise ValueError("ExplorationGoalAttempt data must be a dictionary.")

    goal = _goal_from_dict(data.get("goal"))

    if goal is None:
        return None

    return ExplorationGoalAttempt(
        goal=goal,
        status=str(data.get("status", "active")),
        reason=str(data.get("reason", "")),
        started_at_s=float(data.get("started_at_s", time.time())),
        finished_at_s=(
            None
            if data.get("finished_at_s") is None
            else float(data["finished_at_s"])
        ),
    )


def _event_from_dict(data: dict[str, Any]) -> ExplorationEvent:
    return ExplorationEvent(
        event=str(data.get("event", "")),
        message=str(data.get("message", "")),
        timestamp_s=float(data.get("timestamp_s", time.time())),
        data=dict(data.get("data", {})),
    )


def _counters_from_dict(data: dict[str, Any]) -> ExplorationCounters:
    return ExplorationCounters(
        selected_goals=int(data.get("selected_goals", 0)),
        reached_goals=int(data.get("reached_goals", 0)),
        failed_goals=int(data.get("failed_goals", 0)),
        no_candidate_cycles=int(data.get("no_candidate_cycles", 0)),
        pause_count=int(data.get("pause_count", 0)),
        error_count=int(data.get("error_count", 0)),
    )


def _attempt_event(
    attempt: Optional[ExplorationGoalAttempt],
) -> Optional[ExplorationEvent]:
    if attempt is None:
        return None

    return ExplorationEvent(
        EVENT_GOAL_ATTEMPT_FINISHED,
        attempt.status,
        data=attempt.to_dict(),
    )


def main() -> None:
    goal = ExplorationGoal(
        x=2.0,
        y=3.0,
        yaw=0.0,
        frame_id="map",
        score=0.9,
        reason="frontier_selected",
    )

    state = make_initial_exploration_state()
    state = state.start()
    state = state.select_goal(goal)
    state = state.mark_goal_failed("nav2_failed")

    print(state.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "EVENT_COMPLETED",
    "EVENT_ERROR",
    "EVENT_GOAL_ATTEMPT_FINISHED",
    "EVENT_GOAL_FAILED",
    "EVENT_GOAL_REACHED",
    "EVENT_GOAL_SELECTED",
    "EVENT_PAUSED",
    "EVENT_RESUMED",
    "EVENT_STARTED",
    "EVENT_STOPPED",
    "EVENT_WAITING",
    "STATE_COMPLETE",
    "STATE_ERROR",
    "STATE_IDLE",
    "STATE_PAUSED",
    "STATE_RUNNING",
    "STATE_STOPPED",
    "STATE_WAITING",
    "ExplorationCounters",
    "ExplorationEvent",
    "ExplorationGoalAttempt",
    "ExplorationState",
    "exploration_state_from_dict",
    "make_initial_exploration_state",
]