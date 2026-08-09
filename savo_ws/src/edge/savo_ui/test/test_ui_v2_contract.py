# Copyright 2026 Ahnaf Tahmid
"""Static integration contracts for the non-destructive Savo UI V2."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_ui_v2_new_files_exist():
    required = [
        'include/savo_ui/v2/ui_v2_types.hpp',
        'include/savo_ui/v2/graphics.hpp',
        'include/savo_ui/v2/voice_face_animation.hpp',
        'include/savo_ui/v2/voice_face_renderer.hpp',
        'include/savo_ui/v2/navigation_renderer.hpp',
        'include/savo_ui/v2/ui_v2_node.hpp',
        'src/v2/graphics.cpp',
        'src/v2/voice_face_animation.cpp',
        'src/v2/voice_face_renderer.cpp',
        'src/v2/navigation_renderer.cpp',
        'src/v2/ui_v2_node.cpp',
        'src/v2/main_ui_v2.cpp',
        'config/ui_v2.yaml',
        'config/profiles/pc_dryrun.yaml',
        'config/profiles/edge_real_robot_v1.yaml',
        'launch/ui_v2_bringup.launch.py',
        'launch/ui_select.launch.py',
    ]

    for relative in required:
        assert (ROOT / relative).is_file(), relative


def test_ui_v2_is_read_only_ros_observer():
    source = (ROOT / 'src/v2/ui_v2_node.cpp').read_text(
        encoding='utf-8'
    )

    assert 'create_subscription<std_msgs::msg::String>' in source
    assert 'create_publisher' not in source


def test_ui_v2_voice_and_navigation_topics_are_production_topics():
    config = (ROOT / 'config/ui_v2.yaml').read_text(
        encoding='utf-8'
    )

    topics = [
        '/savo_speech/state',
        '/savo_speech/dashboard',
        '/savo_speech/playback/state',
        '/savo_nav/navigation/state',
        '/savo_nav/navigation/status',
        '/savo_nav/navigation/feedback',
        '/savo_nav/navigation/result',
        '/savo_nav/status',
    ]

    for topic in topics:
        assert topic in config


def test_ui_v2_profiles_are_non_destructive():
    pc_profile = (
        ROOT / 'config/profiles/pc_dryrun.yaml'
    ).read_text(encoding='utf-8')

    edge_profile = (
        ROOT / 'config/profiles/edge_real_robot_v1.yaml'
    ).read_text(encoding='utf-8')

    assert 'enable_framebuffer: false' in pc_profile
    assert 'export_preview_frames: true' in pc_profile

    assert 'enable_framebuffer: true' in edge_profile
    assert 'export_preview_frames: false' in edge_profile


def test_ui_v2_launch_uses_base_and_profile_configuration():
    launch = (
        ROOT / 'launch/ui_v2_bringup.launch.py'
    ).read_text(encoding='utf-8')

    assert "executable='ui_v2_node'" in launch
    assert "default_value='dryrun'" in launch
    assert "'dryrun': 'pc_dryrun.yaml'" in launch
    assert "'pi': 'edge_real_robot_v1.yaml'" in launch
    assert 'base_config' in launch
    assert 'profile_config' in launch
    assert 'page_mode' in launch


def test_ui_selector_defaults_to_classic_and_can_select_v2():
    selector = (
        ROOT / 'launch/ui_select.launch.py'
    ).read_text(encoding='utf-8')

    assert "default_value='classic'" in selector
    assert 'ui_bringup.launch.py' in selector
    assert 'ui_v2_bringup.launch.py' in selector
    assert 'ui_variant' in selector
