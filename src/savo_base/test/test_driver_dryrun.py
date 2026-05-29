#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from savo_base.drivers.board_factory import make_motor_board


def test_make_dryrun_motor_board():
    board = make_motor_board(
        board_backend="dryrun",
        max_duty=3000,
        debug=False,
    )

    assert board is not None


if __name__ == "__main__":
    test_make_dryrun_motor_board()
    print("Dryrun board test passed!")
