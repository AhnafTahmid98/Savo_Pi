from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _cfg(*parts):
    return PathJoinSubstitution([
        FindPackageShare("savo_power"),
        "config",
        *parts,
    ])


def _module_cmd(module):
    return [
        "python3",
        "-c",
        f"from {module} import main; main()",
    ]


def _enabled(flag):
    return IfCondition(PythonExpression([
        "'", flag, "' == 'true'",
    ]))


def _enabled_on_core(flag, role):
    return IfCondition(PythonExpression([
        "'", flag, "' == 'true' and '", role, "' == 'core'",
    ]))


def generate_launch_description():
    role = LaunchConfiguration("role")
    run_i2c_check = LaunchConfiguration("run_i2c_check")
    run_ups_check = LaunchConfiguration("run_ups_check")
    run_kit_battery_check = LaunchConfiguration("run_kit_battery_check")

    diagnostic_config = _cfg("diagnostic.yaml")

    env = {
        "SAVO_POWER_DIAGNOSTIC_CONFIG": diagnostic_config,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "role",
            default_value="core",
            description="Diagnostic target role: core or edge.",
        ),
        DeclareLaunchArgument(
            "run_i2c_check",
            default_value="true",
            description="Run I2C power-device address check.",
        ),
        DeclareLaunchArgument(
            "run_ups_check",
            default_value="true",
            description="Run UPS HAT read check.",
        ),
        DeclareLaunchArgument(
            "run_kit_battery_check",
            default_value="true",
            description="Run base battery ADS7830 read check on core.",
        ),

        ExecuteProcess(
            cmd=_module_cmd("savo_power.diagnostics.i2c_power_check"),
            output="screen",
            additional_env=env,
            condition=_enabled(run_i2c_check),
        ),
        ExecuteProcess(
            cmd=_module_cmd("savo_power.diagnostics.ups_check"),
            output="screen",
            additional_env=env,
            condition=_enabled(run_ups_check),
        ),
        ExecuteProcess(
            cmd=_module_cmd("savo_power.diagnostics.kit_battery_check"),
            output="screen",
            additional_env=env,
            condition=_enabled_on_core(run_kit_battery_check, role),
        ),
    ])
