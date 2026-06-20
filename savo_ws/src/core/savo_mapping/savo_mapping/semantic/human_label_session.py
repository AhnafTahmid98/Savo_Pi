#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Human label session helpers for Robot Savo semantic mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, replace
from typing import Any, Optional, Sequence

from savo_mapping.semantic.location_candidate import (
    LocationCandidate,
    LocationPose,
    make_location_candidate,
    make_location_key,
)
from savo_mapping.semantic.semantic_landmark_store import (
    SemanticLandmarkStore,
)


SESSION_STATUS_OPEN = "open"
SESSION_STATUS_CONFIRMED = "confirmed"
SESSION_STATUS_REJECTED = "rejected"
SESSION_STATUS_CANCELLED = "cancelled"

ACTION_CONFIRM = "confirm"
ACTION_REJECT = "reject"
ACTION_RENAME = "rename"
ACTION_UPDATE_POSE = "update_pose"
ACTION_CANCEL = "cancel"

VALID_SESSION_STATUSES = (
    SESSION_STATUS_OPEN,
    SESSION_STATUS_CONFIRMED,
    SESSION_STATUS_REJECTED,
    SESSION_STATUS_CANCELLED,
)

VALID_HUMAN_ACTIONS = (
    ACTION_CONFIRM,
    ACTION_REJECT,
    ACTION_RENAME,
    ACTION_UPDATE_POSE,
    ACTION_CANCEL,
)


@dataclass(frozen=True)
class HumanLabelAction:
    action: str
    actor: str = "operator"
    label: str = ""
    reason: str = ""
    pose: Optional[LocationPose] = None
    timestamp_s: float = field(default_factory=time.time)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.action not in VALID_HUMAN_ACTIONS:
            raise ValueError(f"Invalid human label action: {self.action}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "action": self.action,
            "actor": self.actor,
            "label": self.label,
            "reason": self.reason,
            "pose": self.pose.to_dict() if self.pose is not None else None,
            "timestamp_s": self.timestamp_s,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class HumanLabelSession:
    session_id: str
    candidate: LocationCandidate
    status: str = SESSION_STATUS_OPEN
    prompt: str = ""
    actions: tuple[HumanLabelAction, ...] = ()
    created_at_s: float = field(default_factory=time.time)
    updated_at_s: float = field(default_factory=time.time)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.session_id.strip():
            raise ValueError("Human label session_id cannot be empty.")

        if self.status not in VALID_SESSION_STATUSES:
            raise ValueError(f"Invalid human label session status: {self.status}")

    @property
    def open(self) -> bool:
        return self.status == SESSION_STATUS_OPEN

    @property
    def confirmed(self) -> bool:
        return self.status == SESSION_STATUS_CONFIRMED

    @property
    def rejected(self) -> bool:
        return self.status == SESSION_STATUS_REJECTED

    @property
    def cancelled(self) -> bool:
        return self.status == SESSION_STATUS_CANCELLED

    def apply_action(self, action: HumanLabelAction) -> "HumanLabelSession":
        if not self.open:
            raise ValueError("Cannot update a closed human label session.")

        candidate = self.candidate
        status = self.status

        if action.action == ACTION_RENAME:
            if not action.label.strip():
                raise ValueError("Rename action requires a label.")

            candidate = candidate.with_label(action.label)

        elif action.action == ACTION_UPDATE_POSE:
            if action.pose is None:
                raise ValueError("Update-pose action requires a pose.")

            candidate = candidate.with_pose(action.pose)

        elif action.action == ACTION_CONFIRM:
            candidate = candidate.confirm(confirmed_by=action.actor)
            status = SESSION_STATUS_CONFIRMED

        elif action.action == ACTION_REJECT:
            candidate = candidate.reject(action.reason or "rejected_by_operator")
            status = SESSION_STATUS_REJECTED

        elif action.action == ACTION_CANCEL:
            status = SESSION_STATUS_CANCELLED

        return replace(
            self,
            candidate=candidate,
            status=status,
            actions=self.actions + (action,),
            updated_at_s=time.time(),
        )

    def confirm(self, *, actor: str = "operator") -> "HumanLabelSession":
        return self.apply_action(HumanLabelAction(action=ACTION_CONFIRM, actor=actor))

    def reject(
        self,
        reason: str,
        *,
        actor: str = "operator",
    ) -> "HumanLabelSession":
        return self.apply_action(
            HumanLabelAction(
                action=ACTION_REJECT,
                actor=actor,
                reason=reason,
            )
        )

    def rename(
        self,
        label: str,
        *,
        actor: str = "operator",
    ) -> "HumanLabelSession":
        return self.apply_action(
            HumanLabelAction(
                action=ACTION_RENAME,
                actor=actor,
                label=label,
            )
        )

    def update_pose(
        self,
        pose: LocationPose,
        *,
        actor: str = "operator",
    ) -> "HumanLabelSession":
        return self.apply_action(
            HumanLabelAction(
                action=ACTION_UPDATE_POSE,
                actor=actor,
                pose=pose,
            )
        )

    def cancel(self, *, actor: str = "operator") -> "HumanLabelSession":
        return self.apply_action(HumanLabelAction(action=ACTION_CANCEL, actor=actor))

    def to_dict(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "status": self.status,
            "prompt": self.prompt,
            "candidate": self.candidate.to_dict(),
            "actions": [action.to_dict() for action in self.actions],
            "created_at_s": self.created_at_s,
            "updated_at_s": self.updated_at_s,
            "metadata": dict(self.metadata),
            "open": self.open,
            "confirmed": self.confirmed,
            "rejected": self.rejected,
            "cancelled": self.cancelled,
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def make_human_label_session(
    candidate: LocationCandidate,
    *,
    session_id: Optional[str] = None,
    prompt: Optional[str] = None,
    metadata: Optional[dict[str, Any]] = None,
) -> HumanLabelSession:
    clean_session_id = session_id or _make_session_id(candidate)

    return HumanLabelSession(
        session_id=clean_session_id,
        candidate=candidate,
        prompt=prompt or make_confirmation_prompt(candidate),
        metadata=dict(metadata or {}),
    )


def make_confirmation_prompt(candidate: LocationCandidate) -> str:
    return (
        f"Should I save this place as {candidate.label} "
        f"at x={candidate.pose.x:.2f}, y={candidate.pose.y:.2f}?"
    )


def apply_session_to_store(
    store: SemanticLandmarkStore,
    session: HumanLabelSession,
) -> SemanticLandmarkStore:
    updated = store.upsert_candidate(session.candidate)

    if session.confirmed:
        return updated.confirm_candidate(
            session.candidate.key,
            confirmed_by=session.candidate.confirmed_by or "operator",
        )

    if session.rejected:
        return updated.reject_candidate(
            session.candidate.key,
            reason=session.candidate.rejected_reason or "rejected_by_operator",
        )

    return updated


def make_manual_location_session(
    *,
    label: str,
    x: float,
    y: float,
    yaw: float = 0.0,
    frame_id: str = "map",
    actor: str = "operator",
    confidence: float = 1.0,
    area_type: str = "place",
) -> HumanLabelSession:
    candidate = make_location_candidate(
        label=label,
        x=x,
        y=y,
        yaw=yaw,
        frame_id=frame_id,
        source="human",
        confidence=confidence,
        area_type=area_type,
        metadata={"created_by": actor},
    )

    return make_human_label_session(candidate)


def human_label_action_from_dict(data: dict[str, Any]) -> HumanLabelAction:
    pose_data = data.get("pose")
    pose = None

    if pose_data is not None:
        pose = LocationPose(
            x=float(pose_data.get("x", 0.0)),
            y=float(pose_data.get("y", 0.0)),
            yaw=float(pose_data.get("yaw", 0.0)),
            frame_id=str(pose_data.get("frame_id", "map")),
        )

    return HumanLabelAction(
        action=str(data["action"]),
        actor=str(data.get("actor", "operator")),
        label=str(data.get("label", "")),
        reason=str(data.get("reason", "")),
        pose=pose,
        timestamp_s=float(data.get("timestamp_s", time.time())),
        metadata=dict(data.get("metadata", {})),
    )


def human_label_session_from_dict(data: dict[str, Any]) -> HumanLabelSession:
    from savo_mapping.semantic.location_candidate import location_candidate_from_dict

    return HumanLabelSession(
        session_id=str(data["session_id"]),
        candidate=location_candidate_from_dict(data["candidate"]),
        status=str(data.get("status", SESSION_STATUS_OPEN)),
        prompt=str(data.get("prompt", "")),
        actions=tuple(
            human_label_action_from_dict(item)
            for item in data.get("actions", ())
        ),
        created_at_s=float(data.get("created_at_s", time.time())),
        updated_at_s=float(data.get("updated_at_s", time.time())),
        metadata=dict(data.get("metadata", {})),
    )


def _make_session_id(candidate: LocationCandidate) -> str:
    return f"label_{candidate.key}_{int(time.time())}"


def main() -> None:
    session = make_manual_location_session(
        label="A201",
        x=4.2,
        y=8.7,
        actor="operator",
    )

    renamed = session.rename("A201 Entrance")
    confirmed = renamed.confirm(actor="operator")

    store = apply_session_to_store(SemanticLandmarkStore(), confirmed)

    print(confirmed.to_json(indent=2))
    print(store.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "ACTION_CANCEL",
    "ACTION_CONFIRM",
    "ACTION_REJECT",
    "ACTION_RENAME",
    "ACTION_UPDATE_POSE",
    "SESSION_STATUS_CANCELLED",
    "SESSION_STATUS_CONFIRMED",
    "SESSION_STATUS_OPEN",
    "SESSION_STATUS_REJECTED",
    "VALID_HUMAN_ACTIONS",
    "VALID_SESSION_STATUSES",
    "HumanLabelAction",
    "HumanLabelSession",
    "apply_session_to_store",
    "human_label_action_from_dict",
    "human_label_session_from_dict",
    "make_confirmation_prompt",
    "make_human_label_session",
    "make_manual_location_session",
]