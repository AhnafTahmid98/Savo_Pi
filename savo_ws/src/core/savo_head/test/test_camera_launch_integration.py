import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (
        PACKAGE_ROOT / relative_path
    ).read_text(encoding='utf-8')


def test_camera_stack_has_three_exclusive_modes():
    source = read(
        'launch/head_camera_stack.launch.py'
    )

    assert "{'disabled', 'ros', 'udp'}" in source
    assert "if mode == 'disabled':" in source
    assert "if mode == 'ros':" in source

    assert (
        'return [camera_driver, camera_status]'
        in source
    )

    assert 'return [camera_stream]' in source

    assert source.count(
        'head_camera_ros.launch.py'
    ) == 1

    assert source.count(
        'head_camera_stream.launch.py'
    ) == 1


def test_ros_mode_owns_camera_health_node():
    source = read(
        'launch/head_camera_stack.launch.py'
    )

    ros_branch = source.index(
        "if mode == 'ros':"
    )

    status_node = source.index(
        "executable='head_camera_status_node'"
    )

    udp_branch = source.index(
        'camera_stream = IncludeLaunchDescription'
    )

    assert ros_branch < status_node < udp_branch

    assert (
        "namespace='savo_head'"
        in source[ros_branch:udp_branch]
    )

    assert (
        "'camera_health_config_file'"
        in source[ros_branch:udp_branch]
    )


def test_bringup_uses_only_camera_stack():
    source = read(
        'launch/head_bringup.launch.py'
    )

    assert source.count(
        'head_camera_stack.launch.py'
    ) == 1

    assert 'enable_camera_stream' not in source
    assert 'IfCondition' not in source

    required_arguments = {
        '"camera_mode"',
        '"camera_source"',
        '"camera_ros_source_format"',
        '"camera_name"',
        '"camera_frame_id"',
        '"camera_info_url"',
        '"camera_ros_config_file"',
        '"camera_health_config_file"',
        '"camera_width"',
        '"camera_height"',
        '"camera_fps"',
        '"camera_format"',
        '"camera_bitrate_kbps"',
        '"camera_udp_host"',
        '"camera_udp_port"',
    }

    for argument in required_arguments:
        assert argument in source


def test_launch_sources_are_valid_python():
    launch_files = [
        'launch/head_camera_stack.launch.py',
        'launch/head_camera_ros.launch.py',
        'launch/head_camera_stream.launch.py',
        'launch/head_bringup.launch.py',
    ]

    for relative_path in launch_files:
        source = read(relative_path)

        ast.parse(
            source,
            filename=relative_path,
        )


def test_launch_integration_test_is_registered():
    cmake = read('CMakeLists.txt')

    assert (
        'test_camera_launch_integration.py'
        in cmake
    )
