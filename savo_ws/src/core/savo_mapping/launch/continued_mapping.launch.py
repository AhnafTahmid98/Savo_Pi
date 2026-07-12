#!/usr/bin/env python3

import math
import os
import re
import tempfile
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_MAP_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{1,63}$")


def _perform(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()

    if normalized in {"true", "1", "yes", "on"}:
        return True

    if normalized in {"false", "0", "no", "off"}:
        return False

    raise ValueError(f"invalid boolean value: {value}")


def _setup_continued_mapping(context):
    source_root = Path(
        os.path.expanduser(
            _perform(context, "source_map_root")
        )
    ).resolve()

    source_map_id = _perform(
        context,
        "source_map_id",
    )

    output_root = Path(
        os.path.expanduser(
            _perform(context, "output_map_root")
        )
    ).resolve()

    output_map_id = _perform(
        context,
        "output_map_id",
    )

    resume_mode = _perform(
        context,
        "resume_mode",
    )

    allow_overwrite = _parse_bool(
        _perform(
            context,
            "allow_output_overwrite",
        )
    )

    if not _MAP_ID_PATTERN.fullmatch(
        source_map_id
    ):
        return [
            LogInfo(
                msg="Invalid source_map_id"
            ),
            EmitEvent(
                event=Shutdown(
                    reason="invalid source_map_id"
                )
            ),
        ]

    if not _MAP_ID_PATTERN.fullmatch(
        output_map_id
    ):
        return [
            LogInfo(
                msg="Invalid output_map_id"
            ),
            EmitEvent(
                event=Shutdown(
                    reason="invalid output_map_id"
                )
            ),
        ]

    if (
        source_root == output_root
        and source_map_id == output_map_id
        and not allow_overwrite
    ):
        return [
            LogInfo(
                msg=(
                    "Source and output map IDs are equal. "
                    "Use a new output_map_id or explicitly "
                    "enable overwrite."
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason="source output collision"
                )
            ),
        ]

    if resume_mode not in {
        "first_node",
        "given_pose",
    }:
        return [
            LogInfo(
                msg=(
                    "resume_mode must be first_node "
                    "or given_pose"
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason="invalid resume mode"
                )
            ),
        ]

    source_session_directory = (
        source_root / source_map_id
    )

    posegraph_base = (
        source_session_directory /
        source_map_id
    )

    base_params_path = Path(
        _perform(
            context,
            "slam_params_file",
        )
    )

    with base_params_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        parameters = yaml.safe_load(stream)

    slam_parameters = parameters[
        "slam_toolbox"
    ]["ros__parameters"]

    slam_parameters["map_file_name"] = str(
        posegraph_base
    )

    if resume_mode == "first_node":
        slam_parameters[
            "map_start_at_dock"
        ] = True

        # An empty YAML sequence is not a valid Jazzy parameter value.
        # Omit the parameter and let slam_toolbox declare its empty
        # std::vector<double> default.
        slam_parameters.pop(
            "map_start_pose",
            None,
        )
    else:
        pose = [
            float(
                _perform(
                    context,
                    "resume_x",
                )
            ),
            float(
                _perform(
                    context,
                    "resume_y",
                )
            ),
            float(
                _perform(
                    context,
                    "resume_yaw",
                )
            ),
        ]

        if not all(
            math.isfinite(value)
            for value in pose
        ):
            return [
                LogInfo(
                    msg="Resume pose must be finite"
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="invalid resume pose"
                    )
                ),
            ]

        slam_parameters[
            "map_start_at_dock"
        ] = False

        slam_parameters[
            "map_start_pose"
        ] = pose

    temporary = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=(
            "savo_mapping_continued_"
            f"{source_map_id}_"
        ),
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )

    with temporary:
        yaml.safe_dump(
            parameters,
            temporary,
            sort_keys=False,
        )

    temporary_path = temporary.name

    verifier = Node(
        package="savo_mapping",
        executable="saved_map_verifier_node",
        name="saved_map_verifier_node",
        output="screen",
        parameters=[
            {
                "source.session_directory":
                    str(source_session_directory),
                "source.map_id":
                    source_map_id,
                "source.expected_frame":
                    "map",
            }
        ],
    )

    manual_mapping = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(
                        "savo_mapping"
                    ),
                    "launch",
                    "manual_mapping.launch.xml",
                ]
            )
        ),
        launch_arguments={
            "map_id":
                output_map_id,
            "map_output_root":
                str(output_root),
            "allow_map_overwrite":
                str(allow_overwrite).lower(),
            "use_sim_time":
                _perform(
                    context,
                    "use_sim_time",
                ),
            "autostart":
                _perform(
                    context,
                    "autostart",
                ),
            "slam_params_file":
                temporary_path,
            "use_lifecycle_manager":
                _perform(
                    context,
                    "use_lifecycle_manager",
                ),
        }.items(),
    )

    def _after_verification(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "Saved-map preflight failed; "
                        "continued mapping will not start."
                    )
                ),
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            "saved-map verification failed"
                        )
                    )
                ),
            ]

        return [
            LogInfo(
                msg=(
                    "Saved-map preflight passed; "
                    "starting continued mapping."
                )
            ),
            manual_mapping,
        ]

    def _cleanup(_context):
        try:
            Path(temporary_path).unlink(
                missing_ok=True
            )
        except OSError:
            pass

        return []

    verification_handler = (
        RegisterEventHandler(
            OnProcessExit(
                target_action=verifier,
                on_exit=_after_verification,
            )
        )
    )

    cleanup_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=_cleanup
                )
            ]
        )
    )

    return [
        verification_handler,
        cleanup_handler,
        verifier,
    ]


def generate_launch_description():
    package_share = FindPackageShare(
        "savo_mapping"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source_map_root",
                default_value=(
                    "~/Savo_Pi/runtime/maps"
                ),
            ),
            DeclareLaunchArgument(
                "source_map_id",
                default_value="",
            ),
            DeclareLaunchArgument(
                "output_map_root",
                default_value=(
                    "~/Savo_Pi/runtime/maps"
                ),
            ),
            DeclareLaunchArgument(
                "output_map_id",
                default_value=[
                    LaunchConfiguration(
                        "source_map_id"
                    ),
                    TextSubstitution(
                        text="_continued"
                    ),
                ],
            ),
            DeclareLaunchArgument(
                "allow_output_overwrite",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "resume_mode",
                default_value="first_node",
                description=(
                    "first_node or given_pose"
                ),
            ),
            DeclareLaunchArgument(
                "resume_x",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "resume_y",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "resume_yaw",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "use_lifecycle_manager",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "config",
                        "slam_toolbox_mapping.yaml",
                    ]
                ),
            ),
            OpaqueFunction(
                function=_setup_continued_mapping
            ),
        ]
    )
