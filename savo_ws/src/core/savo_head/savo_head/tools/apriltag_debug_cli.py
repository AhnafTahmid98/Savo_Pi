# -*- coding: utf-8 -*-

"""AprilTag semantic confirmation debug CLI for Robot Savo head."""

from __future__ import annotations

import argparse
import json
import time
from typing import Optional

from savo_head.models.semantic_confirmation import (
    AprilTagObservation,
    RobotPoseSnapshot,
    SemanticConfirmationPolicy,
    TagRegistration,
    make_confirmation,
    make_rejection,
)


def parse_aliases(value: str) -> tuple[str, ...]:
    return tuple(item.strip() for item in str(value).split(",") if item.strip())


def make_registration(args: argparse.Namespace) -> Optional[TagRegistration]:
    if not args.label and not args.allow_unknown:
        return None

    label = args.label.strip() if args.label else f"tag_{args.tag_id}"

    return TagRegistration(
        tag_id=args.tag_id,
        label=label,
        confirmation_type=args.confirmation_type,
        enabled=not args.tag_disabled,
        save_as_summon_point=args.summon_point,
        aliases=parse_aliases(args.aliases),
    ).normalized()


def make_observation(args: argparse.Namespace, stamp_s: float) -> AprilTagObservation:
    return AprilTagObservation(
        tag_id=args.tag_id,
        family=args.family,
        confidence=args.confidence,
        distance_m=args.distance,
        stable_frames=args.stable_frames,
        stamp_s=stamp_s - args.age,
        frame_id=args.camera_optical_frame,
        pose_camera_xyz_m=(args.tag_x, args.tag_y, args.tag_z),
        pose_camera_rpy_rad=(args.tag_roll, args.tag_pitch, args.tag_yaw),
    ).normalized()


def make_robot_pose(args: argparse.Namespace, stamp_s: float) -> RobotPoseSnapshot:
    return RobotPoseSnapshot(
        x_m=args.robot_x,
        y_m=args.robot_y,
        yaw_rad=args.robot_yaw,
        frame_id=args.map_frame,
        base_frame=args.base_frame,
        stamp_s=stamp_s,
        linear_speed_mps=args.linear_speed,
        angular_speed_radps=args.angular_speed,
        pose_covariance_xy=args.pose_covariance_xy,
        yaw_covariance=args.yaw_covariance,
        localization_ok=not args.localization_bad,
        lidar_map_pose_ok=not args.lidar_map_pose_bad,
        tf_ok=not args.tf_bad,
    )


def make_policy(args: argparse.Namespace) -> SemanticConfirmationPolicy:
    return SemanticConfirmationPolicy(
        min_stable_frames=args.min_stable_frames,
        min_detection_confidence=args.min_confidence,
        max_detection_distance_m=args.max_distance,
        max_detection_age_s=args.max_age,
        require_tf_available=not args.no_require_tf,
        require_robot_stationary=not args.no_require_stationary,
        max_robot_linear_speed_mps=args.max_linear_speed,
        max_robot_angular_speed_radps=args.max_angular_speed,
        require_localization_ok=not args.no_require_localization,
        max_pose_covariance_xy=args.max_pose_covariance_xy,
        max_yaw_covariance=args.max_yaw_covariance,
        require_lidar_map_pose=not args.no_require_lidar_map_pose,
        require_semantic_label=not args.no_require_label,
    )


def run_debug(args: argparse.Namespace) -> int:
    now_s = time.monotonic()

    registration = make_registration(args)
    observation = make_observation(args, stamp_s=now_s)
    robot_pose = make_robot_pose(args, stamp_s=now_s)
    policy = make_policy(args)

    reasons = policy.rejection_reasons(
        observation=observation,
        robot_pose=robot_pose,
        registration=registration,
        now_s=now_s,
    )

    if reasons:
        confirmation = make_rejection(
            observation,
            stamp_s=now_s,
            reasons=reasons,
        )
    else:
        assert registration is not None
        confirmation = make_confirmation(
            observation,
            robot_pose,
            registration,
            stamp_s=now_s,
            reason="apriltag_debug_cli",
        )

    payload = {
        "ok": not reasons,
        "registration": registration.to_dict() if registration else None,
        "observation": observation.to_dict(),
        "robot_pose": robot_pose.to_dict(),
        "rejection_reasons": reasons,
        "confirmation": confirmation.to_dict(),
    }

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print("\nRobot Savo — AprilTag Debug")
        print("---------------------------")
        print(f"Tag ID       : {observation.tag_id}")
        print(f"Label        : {registration.label if registration else '<unregistered>'}")
        print(f"State        : {confirmation.state}")
        print(f"Confidence   : {observation.confidence:.2f}")
        print(f"Distance     : {observation.distance_m:.2f} m")
        print(f"Stable frames: {observation.stable_frames}")
        print(f"Robot pose   : x={robot_pose.x_m:.3f}, y={robot_pose.y_m:.3f}, yaw={robot_pose.yaw_rad:.3f}")

        if reasons:
            print("Rejected     : " + ", ".join(reasons))
        else:
            print("Confirmed    : semantic confirmation is valid")
            print(f"Save summon  : {confirmation.save_as_summon_point}")

    return 0 if not reasons else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Robot Savo AprilTag semantic confirmation debug CLI"
    )

    parser.add_argument("--tag-id", type=int, required=True)
    parser.add_argument("--label", default="")
    parser.add_argument("--aliases", default="")
    parser.add_argument(
        "--confirmation-type",
        choices=["known_location", "summon_point"],
        default="known_location",
    )
    parser.add_argument("--summon-point", action="store_true")
    parser.add_argument("--tag-disabled", action="store_true")
    parser.add_argument("--allow-unknown", action="store_true")

    parser.add_argument("--family", default="tag36h11")
    parser.add_argument("--confidence", type=float, default=0.90)
    parser.add_argument("--distance", type=float, default=1.20)
    parser.add_argument("--stable-frames", type=int, default=6)
    parser.add_argument("--age", type=float, default=0.10)

    parser.add_argument("--camera-optical-frame", default="pi_camera_optical_frame")
    parser.add_argument("--tag-x", type=float, default=0.0)
    parser.add_argument("--tag-y", type=float, default=0.0)
    parser.add_argument("--tag-z", type=float, default=1.0)
    parser.add_argument("--tag-roll", type=float, default=0.0)
    parser.add_argument("--tag-pitch", type=float, default=0.0)
    parser.add_argument("--tag-yaw", type=float, default=0.0)

    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--robot-x", type=float, default=0.0)
    parser.add_argument("--robot-y", type=float, default=0.0)
    parser.add_argument("--robot-yaw", type=float, default=0.0)

    parser.add_argument("--linear-speed", type=float, default=0.0)
    parser.add_argument("--angular-speed", type=float, default=0.0)
    parser.add_argument("--pose-covariance-xy", type=float, default=0.03)
    parser.add_argument("--yaw-covariance", type=float, default=0.02)

    parser.add_argument("--tf-bad", action="store_true")
    parser.add_argument("--localization-bad", action="store_true")
    parser.add_argument("--lidar-map-pose-bad", action="store_true")

    parser.add_argument("--min-stable-frames", type=int, default=5)
    parser.add_argument("--min-confidence", type=float, default=0.70)
    parser.add_argument("--max-distance", type=float, default=3.0)
    parser.add_argument("--max-age", type=float, default=0.50)

    parser.add_argument("--max-linear-speed", type=float, default=0.03)
    parser.add_argument("--max-angular-speed", type=float, default=0.05)
    parser.add_argument("--max-pose-covariance-xy", type=float, default=0.25)
    parser.add_argument("--max-yaw-covariance", type=float, default=0.20)

    parser.add_argument("--no-require-tf", action="store_true")
    parser.add_argument("--no-require-stationary", action="store_true")
    parser.add_argument("--no-require-localization", action="store_true")
    parser.add_argument("--no-require-lidar-map-pose", action="store_true")
    parser.add_argument("--no-require-label", action="store_true")

    parser.add_argument("--json", action="store_true")

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.tag_id < 0:
        parser.error("--tag-id must be non-negative")

    if not 0.0 <= args.confidence <= 1.0:
        parser.error("--confidence must be in range 0..1")

    if args.distance < 0.0:
        parser.error("--distance must be non-negative")

    if args.stable_frames < 0:
        parser.error("--stable-frames must be non-negative")

    return run_debug(args)


if __name__ == "__main__":
    raise SystemExit(main())
