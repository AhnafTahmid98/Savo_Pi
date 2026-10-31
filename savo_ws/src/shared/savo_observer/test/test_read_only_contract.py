"""Fail if observer runtime gains a robot mutation interface."""

import ast
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = [
    ROOT / 'src',
    ROOT / 'launch',
    ROOT / 'rviz',
    ROOT / 'dashboard',
    ROOT / 'savo_observer',
]
PROHIBITED = {
    'create_client<', 'create_service<', 'rclcpp_action', 'async_send_goal',
    'create_client(', 'create_service(', 'ActionClient', 'SetGoal',
    'SetInitialPose', 'PublishPoint', 'Teleop',
}


def test_runtime_has_no_service_action_or_unsafe_rviz_api():
    for directory in RUNTIME:
        for path in directory.rglob('*'):
            if (
                not path.is_file()
                or '__pycache__' in path.parts
                or path.suffix in {'.pyc', '.pyo'}
            ):
                continue

            try:
                source = path.read_text(encoding='utf-8')
            except UnicodeDecodeError:
                continue

            for token in PROHIBITED:
                assert token not in source, (path, token)


def _assert_observer_topic(topic):
    assert isinstance(topic, str), topic
    assert re.fullmatch(r'/savo_observer/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*', topic), topic


def _assert_cpp_publishers(source, allow_output_namespace=False):
    calls = re.findall(r'\bcreate_publisher\s*(?:<[^;]+?>)?\s*\(', source)
    topics = re.findall(
        r'\bcreate_publisher\s*(?:<[^;]+?>)?\s*\(\s*([^,]+),', source
    )
    assert len(topics) == len(calls), 'Unrecognized C++ publisher construction'
    for expression in topics:
        expression = expression.strip()
        if allow_output_namespace and re.fullmatch(
            r'output_namespace\s*\+\s*"/[A-Za-z0-9_/]+"', expression
        ):
            topic = '/savo_observer' + ast.literal_eval(expression.split('+', 1)[1].strip())
        else:
            assert re.fullmatch(r'"[^"\\]*"', expression), expression
            topic = ast.literal_eval(expression)
        _assert_observer_topic(topic)
    return len(topics)


def _assert_python_publishers(source):
    tree = ast.parse(source)
    constants = {
        target.id: node.value
        for node in tree.body if isinstance(node, ast.Assign)
        for target in node.targets if isinstance(target, ast.Name)
    }
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    declarations = [
        call for call in calls
        if isinstance(call.func, ast.Attribute) and call.func.attr == 'declare_parameter'
        and len(call.args) >= 2 and isinstance(call.args[0], ast.Constant)
    ]

    def resolve_topic(node):
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.Name) and node.id in constants:
            # Resolve only a single module constant, never a shadowed binding.
            stores = [
                value for value in ast.walk(tree)
                if isinstance(value, ast.Name) and value.id == node.id
                and isinstance(value.ctx, (ast.Store, ast.Del))
            ]
            assert len(stores) == 1, f'Ambiguous publisher topic constant: {node.id}'
            assert not any(
                isinstance(value, ast.arg) and value.arg == node.id
                for value in ast.walk(tree)
            ), f'Shadowed publisher topic constant: {node.id}'
            return resolve_topic(constants[node.id])
        if isinstance(node, ast.Attribute) and node.attr == 'value':
            call = node.value
            if (
                isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                and call.func.attr == 'get_parameter' and len(call.args) == 1
                and isinstance(call.args[0], ast.Constant)
            ):
                defaults = [
                    declaration.args[1] for declaration in declarations
                    if declaration.args[0].value == call.args[0].value
                ]
                assert len(defaults) == 1, 'Publisher parameter must have one unambiguous default'
                return resolve_topic(defaults[0])
        raise AssertionError(f'Unrecognized Python publisher topic: {ast.dump(node)}')

    publishers = [
        call for call in calls
        if isinstance(call.func, ast.Attribute) and call.func.attr == 'create_publisher'
    ]
    for call in publishers:
        topic = call.args[1] if len(call.args) >= 2 else next(
            (keyword.value for keyword in call.keywords if keyword.arg == 'topic'), None
        )
        assert topic is not None, 'Publisher topic must be explicit'
        _assert_observer_topic(resolve_topic(topic))
    return len(publishers)


def test_ros_publishers_are_confined_to_observer_outputs():
    telemetry = (ROOT / 'src/nodes/observer_telemetry_node.cpp').read_text(
        encoding='utf-8'
    )
    assert 'create_publisher<' in telemetry
    assert '"output_namespace", "/savo_observer"' in telemetry
    assert (
        'const auto validation = savo_observer::ValidateOutputNamespace(output_namespace); '
        'if (!validation.empty()) { throw std::invalid_argument(validation); }'
    ) in ' '.join(telemetry.split())
    for directory in RUNTIME:
        for path in directory.rglob('*'):
            if path.suffix not in {'.cpp', '.hpp', '.py'} or '__pycache__' in path.parts:
                continue
            source = path.read_text(encoding='utf-8')
            if path.suffix == '.py':
                count = _assert_python_publishers(source)
            else:
                count = _assert_cpp_publishers(
                    source, allow_output_namespace=path.name == 'observer_telemetry_node.cpp'
                )
            if 'dashboard' in path.name:
                assert count == 0, path


def test_publisher_boundary_permits_read_only_subscriptions():
    for topic in ('/safety/stop', '/safety/slowdown_factor', '/savo_perception/range/left_m'):
        assert _assert_cpp_publishers(
            f'create_subscription<Float32>("{topic}", qos, callback);'
        ) == 0
        assert _assert_python_publishers(
            f'node.create_subscription(Float32, "{topic}", callback, qos)'
        ) == 0


def test_publisher_boundary_rejects_robot_owned_or_unknown_destinations():
    for topic in (
        '/cmd_vel', '/cmd_vel_manual', '/cmd_vel_nav', '/goal_pose', '/initialpose',
        '/mode_cmd', '/savo_control/mode_cmd', '/safety/stop', '/safety/slowdown_factor',
        '/safety/reset', '/savo_nav/navigation/navigate_to_pose',
        '/savo_mapping/autonomous/run', '/savo_mapping/map_release/promote',
        '/savo_supervisor/authorize_location_operation', '/savo_locations/candidates/approve',
        '/savo_perception/range/left_m', '/savo_observer_other/state',
    ):
        for validate, source in (
            (_assert_cpp_publishers, f'create_publisher<String>("{topic}", qos);'),
            (_assert_python_publishers, f'node.create_publisher(String, "{topic}", qos)'),
            (_assert_cpp_publishers, 'create_publisher<String>(unknown_topic, qos);'),
            (_assert_python_publishers, 'node.create_publisher(String, unknown_topic, qos)'),
            (_assert_python_publishers,
                f'TOPIC = "{topic}"\n'
                'node.declare_parameter("markers_topic", TOPIC)\n'
                'node.create_publisher(String, node.get_parameter("markers_topic").value, 10)'),
            (_assert_python_publishers,
                'TOPIC = "/savo_observer/state"\n'
                f'def unsafe():\n    TOPIC = "{topic}"\n'
                '    node.create_publisher(String, TOPIC, 10)'),
            (_assert_python_publishers,
                'class Unsafe:\n    def setup(self):\n'
                f'        self.declare_parameter("markers_topic", "{topic}")\n'
                '        self.create_publisher(\n'
                '            String, self.get_parameter("markers_topic").value, 10)\n'
                'class Safe:\n    def setup(self):\n'
                '        self.declare_parameter("markers_topic", "/savo_observer/state")'),
        ):
            try:
                validate(source)
            except AssertionError:
                continue
            raise AssertionError(
                f'Robot-owned or unknown publisher destination accepted: {source}'
            )


def test_runtime_does_not_use_stale_realsense_status_topic():
    for directory in RUNTIME:
        for path in directory.rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts:
                continue
            try:
                source = path.read_text(encoding='utf-8')
            except UnicodeDecodeError:
                continue
            assert '/savo_realsense/status' not in source, path
