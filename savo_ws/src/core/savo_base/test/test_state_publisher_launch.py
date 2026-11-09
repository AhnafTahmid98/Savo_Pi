"""Execute the actual launch factory with only ROS launch transport substituted."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

PACKAGE = Path(__file__).resolve().parents[1]


def factory():
    tree = ast.parse((PACKAGE / "launch/base_bringup.launch.py").read_text())
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name == "_build_base_state_publisher_node"]
    assert functions, "Missing exclusive state publisher launch factory"

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    class Condition:
        def __init__(self, value):
            self.value = value

        def evaluate(self, context):
            return self.value.perform(context) == "true"

    namespace = {"LaunchConfiguration": Configuration, "IfCondition": Condition,
                 "Node": lambda **kwargs: SimpleNamespace(**kwargs)}
    exec(compile(ast.Module(body=functions, type_ignores=[]), "launch_factory", "exec"), namespace)
    return namespace["_build_base_state_publisher_node"]


@pytest.mark.parametrize("impl,executable", [("py", "base_state_publisher_node.py"),
                                             ("cpp", "base_state_publisher_node_cpp")])
@pytest.mark.parametrize("enabled", ["true", "false"])
def test_exclusive_selection(impl, executable, enabled):
    nodes = factory()({"base_state_publisher_impl": impl, "use_state_publisher": enabled,
                       "output": "screen", "log_level": "debug"})
    assert len(nodes) == (1 if enabled == "true" else 0)
    if nodes:
        assert nodes[0].executable == executable
        assert nodes[0].name == "base_state_publisher_node"
        assert nodes[0].parameters[0]["cmd_stale_s"] == 0.30
        assert nodes[0].parameters[0]["subscribe_motor_board_status"] is False


def test_invalid_selection_rejected():
    with pytest.raises(RuntimeError, match="base_state_publisher_impl"):
        factory()({"base_state_publisher_impl": "typo", "use_state_publisher": "true"})


@pytest.mark.parametrize("relative", [
    "core/savo_base/launch/base_bringup.launch.py",
    "core/savo_base/launch/base_safe_idle.launch.py",
    "core/savo_base/launch/base_dryrun_test.launch.py",
    "core/savo_base/launch/base_teleop_test.launch.py",
    "shared/savo_bringup/launch/core_bringup.launch.py",
    "shared/savo_bringup/launch/autonomous_mapping.launch.py",
])
def test_launch_defaults_and_forwarding(relative):
    tree = ast.parse((PACKAGE.parents[1] / relative).read_text())
    declarations = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Name) and n.func.id == "DeclareLaunchArgument"
                    and n.args and isinstance(n.args[0], ast.Constant)
                    and n.args[0].value == "base_state_publisher_impl"]
    assert len(declarations) == 1
    keywords = {kw.arg: ast.literal_eval(kw.value) for kw in declarations[0].keywords}
    assert keywords["default_value"] == "py"
    assert set(keywords["choices"]) == {"py", "cpp"}
    if "base_bringup.launch.py" not in relative:
        # Verify the argument is explicitly passed to the included launch.
        assert any(isinstance(n, ast.Dict) and any(
            isinstance(k, ast.Constant) and k.value == "base_state_publisher_impl" for k in n.keys)
            for n in ast.walk(tree))
