#!/usr/bin/env python3
"""Wheels-raised motor direction test through approved Robot SAVO control input."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import argparse
import json
import math
import time
from datetime import UTC, datetime

from tools.diag.infra.diag_utils import emit_result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allow-motion', action='store_true')
    parser.add_argument('--wheels-raised', action='store_true')
    parser.add_argument('--direction', choices=('forward', 'backward', 'left', 'right', 'rotate_left', 'rotate_right'), required=True)
    parser.add_argument('--speed', type=float, default=0.10)
    parser.add_argument('--duration', type=float, default=0.5)
    parser.add_argument('--output', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args(); started=time.monotonic(); started_utc=datetime.now(UTC).isoformat()
    if not args.allow_motion or not args.wheels_raised:
        return emit_result('motor_direction', 'BLOCKED', 'requires_allow_motion_and_wheels_raised', {},
                           output=args.output, started=started, started_utc=started_utc)
    if not 0.04 <= args.speed <= 0.12 or not 0.1 <= args.duration <= 1.0:
        return emit_result('motor_direction', 'FAIL', 'speed_or_duration_out_of_safe_test_bounds',
                           {'speed': args.speed, 'duration': args.duration}, output=args.output,
                           started=started, started_utc=started_utc)
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from std_msgs.msg import Bool, String
    except ImportError as exc:
        return emit_result('motor_direction', 'BLOCKED', 'ros_runtime_unavailable', {'error': str(exc)},
                           output=args.output, started=started, started_utc=started_utc)

    rclpy.init(args=None); node=Node('robot_savo_motor_direction_test')
    state={'mode': None, 'safety': None, 'external_stop': None}
    node.create_subscription(String, '/savo_control/mode_state', lambda m: state.__setitem__('mode', m.data), 10)
    node.create_subscription(Bool, '/safety/stop', lambda m: state.__setitem__('safety', m.data), 10)
    node.create_subscription(Bool, '/savo_control/external_stop', lambda m: state.__setitem__('external_stop', m.data), 10)
    publisher=node.create_publisher(Twist, '/cmd_vel_manual', 10)
    preflight_deadline=time.monotonic()+3.0
    while time.monotonic()<preflight_deadline and None in state.values(): rclpy.spin_once(node, timeout_sec=0.1)
    details={'preflight': state.copy(), 'direction': args.direction, 'speed': args.speed, 'duration': args.duration}
    if state['mode'] is None or 'MANUAL' not in str(state['mode']).upper():
        status,reason='BLOCKED','control_mode_not_manual'
    elif state['safety'] is not False or state['external_stop'] is not False:
        status,reason='BLOCKED','safety_or_external_stop_not_confirmed_clear'
    else:
        command=Twist()
        if args.direction == 'forward': command.linear.x=args.speed
        elif args.direction == 'backward': command.linear.x=-args.speed
        elif args.direction == 'left': command.linear.y=args.speed
        elif args.direction == 'right': command.linear.y=-args.speed
        elif args.direction == 'rotate_left': command.angular.z=min(0.35, args.speed*3.0)
        else: command.angular.z=-min(0.35, args.speed*3.0)
        end=time.monotonic()+args.duration
        try:
            while time.monotonic()<end:
                rclpy.spin_once(node, timeout_sec=0.0)
                if state['safety'] is not False or state['external_stop'] is not False:
                    raise RuntimeError('safety_changed_during_test')
                publisher.publish(command); time.sleep(0.05)
            status,reason='PASS','bounded_manual_command_completed'
        except RuntimeError as exc:
            status,reason='FAIL',str(exc)
        finally:
            zero=Twist()
            for _ in range(5): publisher.publish(zero); time.sleep(0.05)
    node.destroy_node(); rclpy.shutdown()
    return emit_result('motor_direction',status,reason,details,output=args.output,started=started,started_utc=started_utc)


if __name__ == '__main__':
    raise SystemExit(main())
