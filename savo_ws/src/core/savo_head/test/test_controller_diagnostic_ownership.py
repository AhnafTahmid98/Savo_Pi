from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (
        PACKAGE_ROOT / relative_path
    ).read_text(encoding='utf-8')


def test_cpp_controller_uses_owned_topics():
    head_types = read(
        'include/savo_head/core/head_types.hpp'
    )

    controller = read(
        'src/nodes/head_controller_node.cpp'
    )

    assert (
        'kTopicControllerStatus'
        in head_types
    )

    assert (
        '"/savo_head/controller/status"'
        in head_types
    )

    assert (
        'kTopicControllerDashboardText'
        in head_types
    )

    assert (
        '"/savo_head/controller/dashboard_text"'
        in head_types
    )

    assert (
        '"status_topic",\n'
        '      kTopicControllerStatus'
        in controller
    )

    assert (
        '"dashboard_text_topic",\n'
        '      kTopicControllerDashboardText'
        in controller
    )


def test_cpp_controller_formats_const_char_as_text():
    controller = read(
        'src/nodes/head_controller_node.cpp'
    )

    required = {
        'const char * value',
        'value != nullptr ? value : ""',
        'kv("mode", to_string(state.mode))',
        'kv("state_status", to_string(state.status))',
    }

    for fragment in required:
        assert fragment in controller, (
            f'Missing controller formatting contract: '
            f'{fragment}'
        )


def test_python_fallback_uses_owned_topics():
    topics = read(
        'savo_head/contracts/topic_names.py'
    )

    controller = read(
        'savo_head/nodes/head_controller_node.py'
    )

    assert (
        'CONTROLLER_STATUS'
        in topics
    )

    assert (
        'CONTROLLER_DASHBOARD_TEXT'
        in topics
    )

    assert (
        '"/savo_head/controller/status"'
        in topics
    )

    assert (
        '"/savo_head/controller/dashboard_text"'
        in topics
    )

    assert (
        '"status_topic",\n'
        '            CONTROLLER_STATUS'
        in controller
    )

    assert (
        '"dashboard_text_topic",\n'
        '            CONTROLLER_DASHBOARD_TEXT'
        in controller
    )


def test_bringup_locks_topic_ownership():
    launch = read(
        'launch/head_bringup.launch.py'
    )

    assert (
        launch.count(
            '"/savo_head/controller/status"'
        )
        == 1
    )

    assert (
        launch.count(
            '"/savo_head/controller/dashboard_text"'
        )
        == 1
    )

    assert (
        launch.count(
            '"status_topic": "/savo_head/status"'
        )
        == 1
    )

    assert (
        launch.count(
            '"/savo_head/dashboard_text"'
        )
        >= 1
    )


def test_configuration_documents_ownership():
    head_topics = yaml.safe_load(
        read('config/head_topics.yaml')
    )['savo_head']['ros__parameters']

    diagnostics = yaml.safe_load(
        read('config/diagnostics.yaml')
    )['savo_head']['ros__parameters']

    for params in (head_topics, diagnostics):
        assert (
            params['status_topic']
            == '/savo_head/status'
        )

        assert (
            params['dashboard_text_topic']
            == '/savo_head/dashboard_text'
        )

        assert (
            params['controller_status_topic']
            == '/savo_head/controller/status'
        )

        assert (
            params[
                'controller_dashboard_text_topic'
            ]
            == (
                '/savo_head/controller/'
                'dashboard_text'
            )
        )


def test_controller_does_not_default_to_aggregate():
    controller = read(
        'src/nodes/head_controller_node.cpp'
    )

    parameter_section = controller[
        controller.index(
            'void declare_parameters()'
        ):
        controller.index(
            'std::unique_ptr<PanTiltDriver>'
        )
    ]

    assert (
        '"status_topic", kTopicStatus'
        not in parameter_section
    )

    assert (
        '"dashboard_text_topic", '
        'kTopicDashboardText'
        not in parameter_section
    )
