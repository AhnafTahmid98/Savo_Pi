from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (
        PACKAGE_ROOT / relative_path
    ).read_text(encoding='utf-8')


def test_head_status_uses_camera_report_parser():
    source = read(
        'src/nodes/head_status_node.cpp'
    )

    required = {
        'camera_status_report.hpp',
        'camera_status_health(',
        'parse_camera_status_text(',
        'camera_state_text(',
        'camera_reason_text(',
        '"camera_status_valid"',
        '"camera_stream_healthy"',
        '"camera_pose_ready"',
        '"camera_calibrated"',
        '"camera_required"',
        'summarize_health(',
        'components,\n      true',
    }

    for fragment in required:
        assert fragment in source, (
            f'Missing camera severity integration: '
            f'{fragment}'
        )


def test_required_component_loss_is_error():
    source = read(
        'src/nodes/head_status_node.cpp'
    )

    assert (
        'required_component_stale'
        in source
    )

    assert (
        'required_component_missing'
        in source
    )

    assert (
        'error_component('
        in source
    )


def test_camera_status_core_is_registered():
    cmake = read('CMakeLists.txt')

    assert (
        'src/core/camera_status_report.cpp'
        in cmake
    )

    assert (
        'test_camera_status_report_core.py'
        in cmake
    )

    assert (
        'test_head_status_camera_severity_contract.py'
        in cmake
    )
