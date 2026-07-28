from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (
        PACKAGE_ROOT / relative_path
    ).read_text(encoding='utf-8')


def test_live_camera_status_node_exists():
    source = read(
        'src/nodes/head_camera_status_node.cpp'
    )

    required_fragments = {
        'class HeadCameraStatusNode',
        'sensor_msgs::msg::Image',
        'sensor_msgs::msg::CameraInfo',
        'evaluate_camera_health(',
        'camera_health_status_text(',
        'image_timestamp_monotonic_',
        'camera_info_timestamp_monotonic_',
        'camera_info_is_calibrated(',
        'frame_rate_ema_alpha_',
        'create_service<std_srvs::srv::Trigger>',
        'savo_head.camera',
    }

    for fragment in required_fragments:
        assert fragment in source, (
            f'Missing node contract: {fragment}'
        )


def test_live_camera_status_topics_are_locked():
    source = read(
        'src/nodes/head_camera_status_node.cpp'
    )

    assert 'kTopicCameraImageRaw' in source
    assert 'kTopicCameraInfo' in source
    assert 'kTopicCameraStatus' in source
    assert 'kFrameCameraOptical' in source


def test_camera_health_yaml_matches_node():
    data = yaml.safe_load(
        read('config/camera_health.yaml')
    )

    params = data[
        '/savo_head/head_camera_status_node'
    ]['ros__parameters']

    assert (
        params['image_topic']
        == '/savo_head/camera/image_raw'
    )

    assert (
        params['camera_info_topic']
        == '/savo_head/camera/camera_info'
    )

    assert (
        params['status_topic']
        == '/savo_head/camera/status'
    )

    assert (
        params['health_check_service']
        == '/savo_head/camera/health_check'
    )

    assert (
        params['expected_frame_id']
        == 'pi_camera_optical_frame'
    )

    assert params['expected_encoding'] == 'rgb8'
    assert params['status_publish_hz'] > 0.0


def test_cmake_builds_live_camera_status_node():
    cmake = read('CMakeLists.txt')

    assert (
        'add_savo_head_node('
        'head_camera_status_node '
        'src/nodes/head_camera_status_node.cpp)'
        in cmake
    )


def test_uncalibrated_zero_camera_info_uses_image_dimensions():
    source = read(
        'src/nodes/head_camera_status_node.cpp'
    )

    required = {
        'use_image_dimensions_for_uncalibrated_info',
        'camera_info_seen_ &&',
        'image_seen_ &&',
        '!camera_calibrated_',
        'camera_info_width_ == 0U',
        'camera_info_height_ == 0U',
        'image_width_ :',
        'image_height_ :',
    }

    for fragment in required:
        assert fragment in source, (
            'Missing uncalibrated CameraInfo '
            f'compatibility contract: {fragment}'
        )

