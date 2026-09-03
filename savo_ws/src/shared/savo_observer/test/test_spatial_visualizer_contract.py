"""Static contracts for bounded read-only spatial marker nodes."""

from pathlib import Path

import pytest

from savo_observer.range_state import range_sample_state


ROOT = Path(__file__).resolve().parents[1]
LOCALIZATION = (
    ROOT / 'savo_observer' / 'localization_visualizer_node.py'
).read_text(encoding='utf-8')
RANGE = (
    ROOT / 'savo_observer' / 'range_visualizer_node.py'
).read_text(encoding='utf-8')


def test_exact_odometry_topics_and_marker_output_are_used():
    for topic in (
        '/odometry/filtered',
        '/wheel/odom',
        '/vo/odom',
        '/savo_observer/localization_markers',
    ):
        assert topic in LOCALIZATION
    assert LOCALIZATION.count('qos_profile_sensor_data') == 4


def test_exact_three_range_topics_and_frames_are_used():
    for topic in (
        '/savo_perception/range/left_m',
        '/savo_perception/range/right_m',
        '/savo_perception/range/front_ultrasonic_m',
    ):
        assert RANGE.count(topic) == 1
    for frame in (
        'tof_left_link',
        'tof_right_link',
        'ultrasonic_front_link',
    ):
        assert RANGE.count(frame) == 1
    assert RANGE.count('qos_profile_sensor_data') == 2
    assert '/savo_observer/range_markers' in RANGE


def test_marker_geometry_and_ids_are_bounded_and_planar():
    assert "markers = [" in LOCALIZATION
    assert "Exactly five markers" in LOCALIZATION
    assert "Exactly twelve deterministic marker IDs" in RANGE
    assert 'Marker.CUBE' not in LOCALIZATION
    assert 'Marker.CYLINDER' not in LOCALIZATION
    assert 'Marker.CUBE' not in RANGE
    assert 'Marker.CYLINDER' not in RANGE
    assert 'pose.position.x = pose.position.x' in LOCALIZATION
    assert 'pose.position.y = pose.position.y' in LOCALIZATION


@pytest.mark.parametrize(
    ('value', 'received', 'now', 'expected'),
    [
        (None, None, 10.0, 'MISSING'),
        (0.4, 8.0, 10.0, 'STALE'),
        *[
            (math_value, 9.5, 10.0, 'INVALID')
            for math_value in (float('nan'), float('inf'), 0.0, -0.1)
        ],
    ],
)
def test_missing_stale_and_invalid_ranges_have_distinct_states(
    value, received, now, expected
):
    assert range_sample_state(value, received, now, 1.0) == expected


def test_fresh_positive_range_is_valid_without_safety_inference():
    assert range_sample_state(0.42, 9.5, 10.0, 1.0) == 'VALID'
    for forbidden in ('/cmd_vel', '/safety/stop', 'slowdown_factor'):
        assert forbidden not in RANGE
