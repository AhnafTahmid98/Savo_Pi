"""ROS launch adapter for the package-owned staged-startup coordinator."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from launch import Action
from launch.actions import EmitEvent, GroupAction, LogError, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


@dataclass(frozen=True)
class StartupStageGroup:
    """Processes released together after the previous stage is stable-ready."""

    name: str
    actions: tuple[Action, ...]


def _gate(stage: str, status_topic: str, log_level) -> Node:
    return Node(
        package="savo_bringup",
        executable="startup_stage_gate_node",
        name=f"startup_gate_{stage}",
        output="screen",
        parameters=[{"target_stage": stage, "status_topic": status_topic}],
        arguments=["--ros-args", "--log-level", log_level],
    )


def build_staged_sequence(
    groups: Iterable[StartupStageGroup],
    *,
    status_topic: str,
    log_level,
) -> list[Action]:
    """Return event handlers plus the first group in a non-blocking stage chain."""
    ordered = list(groups)
    if not ordered:
        raise ValueError("at least one startup stage group is required")
    if len({group.name for group in ordered}) != len(ordered):
        raise ValueError("startup stage names must be unique")

    gates = [_gate(group.name, status_topic, log_level) for group in ordered]
    launch_groups = [
        GroupAction(actions=[*group.actions, gate])
        for group, gate in zip(ordered, gates)
    ]
    handlers: list[Action] = []

    for index, gate in enumerate(gates):
        next_group = launch_groups[index + 1] if index + 1 < len(ordered) else None
        stage_name = ordered[index].name

        def on_exit(event, _context, *, following=next_group, name=stage_name):
            if event.returncode != 0:
                return [
                    LogError(msg=f"Robot Savo startup stage failed: {name}"),
                    EmitEvent(
                        event=Shutdown(
                            reason=f"startup stage {name} failed"
                        )
                    ),
                ]
            return [] if following is None else [following]

        handlers.append(
            RegisterEventHandler(
                OnProcessExit(target_action=gate, on_exit=on_exit)
            )
        )

    return [*handlers, launch_groups[0]]


__all__ = ["StartupStageGroup", "build_staged_sequence"]
