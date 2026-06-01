#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from savo_base.kinematics.mecanum import mix_mecanum


def test_mecanum_forward_all_wheels_same_sign():
    fl, rl, fr, rr = mix_mecanum(
        0.1,
        0.0,
        0.0,
        forward_sign=-1,
        strafe_sign=1,
        rotate_sign=1,
        turn_gain=1.0,
    )

    assert fl == rl == fr == rr


def test_mecanum_strafe_has_cross_pattern():
    fl, rl, fr, rr = mix_mecanum(
        0.0,
        0.1,
        0.0,
        forward_sign=-1,
        strafe_sign=1,
        rotate_sign=1,
        turn_gain=1.0,
    )

    assert fl == rr
    assert rl == fr
    assert fl == -rl


if __name__ == "__main__":
    test_mecanum_forward_all_wheels_same_sign()
    test_mecanum_strafe_has_cross_pattern()
    print("All mecanum mix tests passed!")
