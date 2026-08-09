# Copyright 2026 Ahnaf Tahmid
"""
Static contracts for the always-on hybrid Savo UI runtime.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_runtime_variant_is_separate_from_classic_ui():
    assert (
        ROOT
        / 'include/savo_ui/runtime/ui_runtime_node.hpp'
    ).is_file()

    assert (
        ROOT
        / 'src/runtime/ui_runtime_node.cpp'
    ).is_file()

    assert (
        ROOT
        / 'src/runtime/main_ui_runtime.cpp'
    ).is_file()


def test_runtime_uses_v2_for_interaction_only():
    source = (
        ROOT
        / 'src/runtime/ui_runtime_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'v2_voice_renderer_->render' in source

    assert (
        'v2_navigation_renderer_->render'
        in source
    )

    # Classic pages remain inside the copied runtime.
    assert 'render_home();' in source
    assert 'render_status_page();' in source
    assert 'render_power_page();' in source


def test_voice_completion_policy():
    source = (
        ROOT
        / 'src/runtime/ui_runtime_node.cpp'
    ).read_text(encoding='utf-8')

    assert (
        'TTS finished | returning to Home dashboard'
        in source
    )

    assert (
        'TTS finished | returning to active V2 Navigation'
        in source
    )


def test_intro_launch_uses_one_runtime_process():
    launch = (
        ROOT
        / 'launch/ui_intro.launch.py'
    ).read_text(encoding='utf-8')

    runtime = (
        ROOT
        / 'launch/ui_runtime_bringup.launch.py'
    ).read_text(encoding='utf-8')

    assert 'ui_runtime_bringup.launch.py' in launch

    assert (
        "executable='ui_runtime_node'"
        in runtime
    )

    assert "name='savo_ui_node'" in runtime


def test_system_service_uses_intro_entrypoint():
    service = (
        ROOT
        / 'systemd/savo-ui-runtime.service'
    ).read_text(encoding='utf-8')

    assert 'ui_intro.launch.py' in service
    assert 'Restart=always' in service

    assert (
        'Conflicts=savo-ui.service'
        in service
    )
