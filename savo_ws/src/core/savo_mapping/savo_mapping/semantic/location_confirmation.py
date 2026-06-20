#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Location confirmation helpers for Robot Savo semantic mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Optional

from savo_mapping.semantic.location_candidate import (
    LocationCandidate,
    LocationPose,
    make_location_key,
)
from savo_mapping.semantic.semantic_landmark_store import (
    SemanticLandmarkStore,
    semantic_landmark_from_candidate,
)


CONFIRM_ACTION_CONFIRM = "confirm"
CONFIRM_ACTION_REJECT = "reject"
CONFIRM_ACTION_RENAME = "rename"
CONFIRM_ACTION_UPDATE_POSE = "update_pose"
CONFIRM_ACTION_CANCEL = "cancel"

CONFIRM_SOURCE_OPERATOR = "operator"
CONFIRM_SOURCE_APP = "app"
CONFIRM_SOURCE_VOICE = "voice"
CONFIRM_SOURCE_CLI = "cli"
CONFIRM_SOURCE_AUTO = "auto"

VALID_CONFIRM_ACTIONS = (
    CONFIRM_ACTION_CONFIRM,
    CONFIRM_ACTION_REJECT,
    CONFIRM_ACTION_RENAME,
    CONFIRM_ACTION_UPDATE_POSE,
    CONFIRM_ACTION_CANCEL,
)

VALID_CONFIRM_SOURCES = (
    CONFIRM_SOURCE_OPERATOR,
    CONFIRM_SOURCE_APP,
    CONFIRM_SOURCE_VOICE,
    CONFIRM_SOURCE_CLI,
    CONFIRM_SOURCE_AUTO,
)


@dataclass(frozen=True)
class LocationConfirmation:
    candidate_key: str
    action: str
    actor: str = "operator"
    source: str = CONFIRM_SOURCE_OPERATOR
    label: str = ""
    reason: str = ""
    pose: Optional[LocationPose] = None
    timestamp_s: float = field(default_factory=time.time)
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.candidate_key.strip():
            raise ValueError("Location confirmation candidate_key cannot be empty.")

        if self.action not in VALID_CONFIRM_ACTIONS:
            raise ValueError(f"Invalid location confirmation action: {self.action}")

        if self.source not in VALID_CONFIRM_SOURCES:
            raise ValueError(f"Invalid location confirmation source: {self.source}")

    @property
    def clean_candidate_key(self) -> str:
        return make_location_key(self.candidate_key)

    def to_dict(self) -> dict[str, Any]:
        return {
            "candidate_key": self.candidate_key,
            "clean_candidate_key": self.clean_candidate_key,
            "action": self.action,
            "actor": self.actor,
            "source": self.source,
            "label": self.label,
            "reason": self.reason,
            "pose": self.pose.to_dict() if self.pose is not None else None,
            "timestamp_s": self.timestamp_s,
            "metadata": dict(self.metadata),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


@dataclass(frozen=True)
class LocationConfirmationResult:
    ok: bool
    confirmation: LocationConfirmation
    candidate_before: Optional[LocationCandidate] = None
    candidate_after: Optional[LocationCandidate] = None
    message: str = "Location confirmation not applied."
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "confirmation": self.confirmation.to_dict(),
            "candidate_before": (
                self.candidate_before.to_dict()
                if self.candidate_before is not None
                else None
            ),
            "candidate_after": (
                self.candidate_after.to_dict()
                if self.candidate_after is not None
                else None
            ),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def make_confirm_location(
    candidate_key: str,
    *,
    actor: str = "operator",
    source: str = CONFIRM_SOURCE_OPERATOR,
) -> LocationConfirmation:
    return LocationConfirmation(
        candidate_key=candidate_key,
        action=CONFIRM_ACTION_CONFIRM,
        actor=actor,
        source=source,
    )


def make_reject_location(
    candidate_key: str,
    *,
    reason: str,
    actor: str = "operator",
    source: str = CONFIRM_SOURCE_OPERATOR,
) -> LocationConfirmation:
    return LocationConfirmation(
        candidate_key=candidate_key,
        action=CONFIRM_ACTION_REJECT,
        actor=actor,
        source=source,
        reason=reason,
    )


def make_rename_location(
    candidate_key: str,
    label: str,
    *,
    actor: str = "operator",
    source: str = CONFIRM_SOURCE_OPERATOR,
) -> LocationConfirmation:
    return LocationConfirmation(
        candidate_key=candidate_key,
        action=CONFIRM_ACTION_RENAME,
        actor=actor,
        source=source,
        label=label,
    )


def make_update_location_pose(
    candidate_key: str,
    pose: LocationPose,
    *,
    actor: str = "operator",
    source: str = CONFIRM_SOURCE_OPERATOR,
) -> LocationConfirmation:
    return LocationConfirmation(
        candidate_key=candidate_key,
        action=CONFIRM_ACTION_UPDATE_POSE,
        actor=actor,
        source=source,
        pose=pose,
    )


def make_cancel_location_confirmation(
    candidate_key: str,
    *,
    actor: str = "operator",
    source: str = CONFIRM_SOURCE_OPERATOR,
    reason: str = "cancelled",
) -> LocationConfirmation:
    return LocationConfirmation(
        candidate_key=candidate_key,
        action=CONFIRM_ACTION_CANCEL,
        actor=actor,
        source=source,
        reason=reason,
    )


def apply_confirmation_to_candidate(
    candidate: LocationCandidate,
    confirmation: LocationConfirmation,
) -> LocationConfirmationResult:
    if candidate.key != confirmation.clean_candidate_key:
        return LocationConfirmationResult(
            ok=False,
            confirmation=confirmation,
            candidate_before=candidate,
            candidate_after=candidate,
            message=(
                "Confirmation key does not match candidate: "
                f"{confirmation.clean_candidate_key} != {candidate.key}."
            ),
        )

    try:
        updated = _apply_action(candidate, confirmation)
    except ValueError as exc:
        return LocationConfirmationResult(
            ok=False,
            confirmation=confirmation,
            candidate_before=candidate,
            candidate_after=candidate,
            message=str(exc),
        )

    return LocationConfirmationResult(
        ok=True,
        confirmation=confirmation,
        candidate_before=candidate,
        candidate_after=updated,
        message=f"Location confirmation applied: {confirmation.action}.",
    )


def apply_confirmation_to_store(
    store: SemanticLandmarkStore,
    confirmation: LocationConfirmation,
) -> tuple[SemanticLandmarkStore, LocationConfirmationResult]:
    candidate = store.find_candidate(confirmation.clean_candidate_key)

    if candidate is None:
        result = LocationConfirmationResult(
            ok=False,
            confirmation=confirmation,
            message=f"Location candidate not found: {confirmation.clean_candidate_key}.",
        )
        return store, result

    result = apply_confirmation_to_candidate(candidate, confirmation)

    if not result.ok or result.candidate_after is None:
        return store, result

    updated_store = store.upsert_candidate(result.candidate_after)

    if result.candidate_after.confirmed:
        landmark = semantic_landmark_from_candidate(result.candidate_after)
        updated_store = updated_store.upsert_landmark(landmark)

    return updated_store, result


def location_confirmation_from_dict(data: dict[str, Any]) -> LocationConfirmation:
    pose_data = data.get("pose")
    pose = None

    if pose_data is not None:
        pose = LocationPose(
            x=float(pose_data.get("x", 0.0)),
            y=float(pose_data.get("y", 0.0)),
            yaw=float(pose_data.get("yaw", 0.0)),
            frame_id=str(pose_data.get("frame_id", "map")),
        )

    return LocationConfirmation(
        candidate_key=str(data["candidate_key"]),
        action=str(data["action"]),
        actor=str(data.get("actor", "operator")),
        source=str(data.get("source", CONFIRM_SOURCE_OPERATOR)),
        label=str(data.get("label", "")),
        reason=str(data.get("reason", "")),
        pose=pose,
        timestamp_s=float(data.get("timestamp_s", time.time())),
        metadata=dict(data.get("metadata", {})),
    )


def _apply_action(
    candidate: LocationCandidate,
    confirmation: LocationConfirmation,
) -> LocationCandidate:
    if confirmation.action == CONFIRM_ACTION_CONFIRM:
        return candidate.confirm(confirmed_by=confirmation.actor)

    if confirmation.action == CONFIRM_ACTION_REJECT:
        return candidate.reject(confirmation.reason or "rejected")

    if confirmation.action == CONFIRM_ACTION_RENAME:
        if not confirmation.label.strip():
            raise ValueError("Rename confirmation requires a non-empty label.")

        return candidate.with_label(confirmation.label)

    if confirmation.action == CONFIRM_ACTION_UPDATE_POSE:
        if confirmation.pose is None:
            raise ValueError("Update-pose confirmation requires a pose.")

        return candidate.with_pose(confirmation.pose)

    if confirmation.action == CONFIRM_ACTION_CANCEL:
        return candidate

    raise ValueError(f"Unsupported confirmation action: {confirmation.action}")


def main() -> None:
    from savo_mapping.semantic.location_candidate import (
        make_apriltag_location_candidate,
    )

    candidate = make_apriltag_location_candidate(
        tag_id=21,
        label="A201",
        x=4.2,
        y=8.7,
        confidence=0.95,
    )

    confirmation = make_rename_location(
        "a201",
        "A201 Entrance",
        source=CONFIRM_SOURCE_CLI,
    )

    rename_result = apply_confirmation_to_candidate(candidate, confirmation)

    confirm_result = apply_confirmation_to_candidate(
        rename_result.candidate_after,
        make_confirm_location(
            "a201_entrance",
            source=CONFIRM_SOURCE_CLI,
        ),
    )

    print(rename_result.to_json(indent=2))
    print(confirm_result.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "CONFIRM_ACTION_CANCEL",
    "CONFIRM_ACTION_CONFIRM",
    "CONFIRM_ACTION_REJECT",
    "CONFIRM_ACTION_RENAME",
    "CONFIRM_ACTION_UPDATE_POSE",
    "CONFIRM_SOURCE_APP",
    "CONFIRM_SOURCE_AUTO",
    "CONFIRM_SOURCE_CLI",
    "CONFIRM_SOURCE_OPERATOR",
    "CONFIRM_SOURCE_VOICE",
    "VALID_CONFIRM_ACTIONS",
    "VALID_CONFIRM_SOURCES",
    "LocationConfirmation",
    "LocationConfirmationResult",
    "apply_confirmation_to_candidate",
    "apply_confirmation_to_store",
    "location_confirmation_from_dict",
    "main",
    "make_cancel_location_confirmation",
    "make_confirm_location",
    "make_reject_location",
    "make_rename_location",
    "make_update_location_pose",
]