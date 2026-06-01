#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from savo_base.models.wheel_command import (
    WheelNorm,
    make_robot_savo_wheel_command,
)


def test_make_wheel_command_with_norm_values():
    cmd = make_robot_savo_wheel_command(
        source="test",
        mode="cmd_vel",
        wheel_norm=WheelNorm(fl=0.1, rl=0.2, fr=0.3, rr=0.4),
        note="unit test",
    )

    assert cmd.wheel_norm.fl == 0.1
    assert cmd.wheel_norm.rl == 0.2
    assert cmd.wheel_norm.fr == 0.3
    assert cmd.wheel_norm.rr == 0.4
    assert cmd.header.source == "test"


if __name__ == "__main__":
    test_make_wheel_command_with_norm_values()
    print("Wheel command test passed!")
