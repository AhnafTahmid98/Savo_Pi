import ast
import py_compile
from pathlib import Path


DEV_TOOLS = [
    "tools/dev/battery_log_viewer.py",
    "tools/dev/estimate_kit_soc_curve.py",
]


def read(path):
    return Path(path).read_text(errors="ignore")


def test_dev_tool_files_exist():
    for path in DEV_TOOLS:
        assert Path(path).is_file(), path


def test_dev_tools_compile_as_python():
    for path in DEV_TOOLS:
        py_compile.compile(path, doraise=True)


def test_dev_tools_parse_as_python_ast():
    for path in DEV_TOOLS:
        ast.parse(read(path))


def test_dev_tools_are_not_empty_placeholders():
    for path in DEV_TOOLS:
        text = read(path).strip()

        assert len(text) > 80, path
        assert "def " in text or "class " in text or "if __name__" in text, path


def test_dev_tools_have_main_or_cli_entry_shape():
    for path in DEV_TOOLS:
        text = read(path)

        assert (
            "def main" in text
            or 'if __name__ == "__main__"' in text
            or "argparse" in text
        ), path


def test_dev_tools_do_not_import_ros_runtime():
    for path in DEV_TOOLS:
        text = read(path)

        assert "import rclpy" not in text
        assert "from rclpy" not in text


def test_dev_tools_do_not_access_i2c_hardware_directly():
    for path in DEV_TOOLS:
        text = read(path)

        assert "/dev/i2c" not in text
        assert "smbus.SMBus" not in text
        assert "SMBus(" not in text


def test_dev_tools_do_not_enable_shutdown_or_motor_control():
    combined = "\n".join(read(path) for path in DEV_TOOLS).lower()

    for forbidden in [
        "automatic_shutdown_enabled: true",
        "automatic_shutdown_enabled=true",
        "shutdown:=true",
        "request_shutdown(true)",
        "/cmd_vel",
        "motor_pwm",
        "wheel_speed",
        "pca9685",
    ]:
        assert forbidden not in combined


def test_battery_log_viewer_mentions_log_or_data():
    text = read("tools/dev/battery_log_viewer.py").lower()

    assert "log" in text or "csv" in text or "json" in text


def test_estimate_kit_soc_curve_mentions_voltage_or_soc():
    text = read("tools/dev/estimate_kit_soc_curve.py").lower()

    assert "voltage" in text or "soc" in text or "battery" in text


def test_dev_tools_are_not_installed_as_robot_runtime_nodes():
    cmake = Path("CMakeLists.txt").read_text()
    setup_py = Path("setup.py").read_text()

    for path in DEV_TOOLS:
        name = Path(path).name

        assert f"tools/dev/{name}" not in cmake
        assert name not in setup_py
