# -*- coding: utf-8 -*-

"""Launch-time configuration resolution for the Phase 0D runtime."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Mapping

from savo_speech.ros.param_utils import (
    ConfigError,
    load_and_apply_profile,
)


PHASE0D_SUPPORTED_PROFILES = frozenset(
    {
        "dryrun_no_hardware",
    }
)


class LaunchConfigError(ConfigError):
    """Raised when a launch profile is unsafe for the current phase."""


def normalize_profile_name(value: object) -> str:
    profile = str(value).strip()

    if profile.endswith(".yaml"):
        profile = profile[:-5]

    if not profile:
        raise LaunchConfigError("profile name must not be empty")

    if "/" in profile or "\\" in profile or profile in {".", ".."}:
        raise LaunchConfigError(
            "profile must be a simple profile name, not a path"
        )

    if re.fullmatch(r"[A-Za-z0-9_]+", profile) is None:
        raise LaunchConfigError(
            "profile may contain only letters, numbers, and underscores"
        )

    return profile


def resolve_profile_path(
    config_dir: str | Path,
    profile_name: object,
) -> Path:
    directory = Path(config_dir)
    profile = normalize_profile_name(profile_name)

    path = directory / "profiles" / f"{profile}.yaml"

    if not path.is_file():
        raise LaunchConfigError(
            f"speech profile does not exist: {path}"
        )

    return path


def validate_phase0d_runtime_config(
    runtime: Mapping[str, Mapping[str, Any]],
) -> None:
    audio = runtime["respeaker_audio_node"]
    manager = runtime["speech_manager_node"]

    if audio["enabled"] is not True:
        raise LaunchConfigError(
            "Phase 0D requires respeaker_audio_node.enabled=true"
        )

    if audio["backend"] != "dryrun":
        raise LaunchConfigError(
            "Phase 0D supports only the dryrun audio backend"
        )

    if manager["enabled"] is not True:
        raise LaunchConfigError(
            "Phase 0D requires speech_manager_node.enabled=true"
        )

    unsupported_components = {
        "wake_word_service": runtime["wake_word_service"]["enabled"],
        "local_stt_service": runtime["local_stt_service"]["enabled"],
        "local_tts_service": runtime["local_tts_service"]["enabled"],
        "savomind_bridge_node": runtime[
            "savomind_bridge_node"
        ]["enabled"],
    }

    enabled_unsupported = sorted(
        name
        for name, enabled in unsupported_components.items()
        if enabled
    )

    if enabled_unsupported:
        raise LaunchConfigError(
            "Phase 0D cannot launch unsupported components: "
            f"{enabled_unsupported}"
        )


def build_phase0d_runtime_config(
    config_dir: str | Path,
    profile_name: object,
) -> dict[str, dict[str, Any]]:
    profile = normalize_profile_name(profile_name)

    if profile not in PHASE0D_SUPPORTED_PROFILES:
        raise LaunchConfigError(
            f"profile {profile!r} is not supported in Phase 0D; "
            f"supported profiles: {sorted(PHASE0D_SUPPORTED_PROFILES)}"
        )

    profile_path = resolve_profile_path(config_dir, profile)
    runtime = load_and_apply_profile(config_dir, profile_path)

    validate_phase0d_runtime_config(runtime)
    return runtime


__all__ = [
    "LaunchConfigError",
    "PHASE0D_SUPPORTED_PROFILES",
    "build_phase0d_runtime_config",
    "normalize_profile_name",
    "resolve_profile_path",
    "validate_phase0d_runtime_config",
]
