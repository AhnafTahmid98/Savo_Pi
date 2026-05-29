#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test clamp and scaling functions.
"""

from savo_base.kinematics.scaling import normalized_to_signed_duty


def test_normalized_to_signed_duty_basic():
    assert normalized_to_signed_duty(
        1.0, -1.0, 0.5, -0.5, max_abs_duty=3000
    ) == (3000, -3000, 1500, -1500)


def test_normalized_to_signed_duty_clamps_values():
    assert normalized_to_signed_duty(
        2.0, -2.0, 0.0, 0.25, max_abs_duty=3000
    ) == (3000, -3000, 0, 750)


if __name__ == "__main__":
    test_normalized_to_signed_duty_basic()
    test_normalized_to_signed_duty_clamps_values()
    print("All scaling tests passed!")
