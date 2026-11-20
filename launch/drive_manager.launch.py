import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from drive_manager.launch_guard import acquire_launch_lock


def configured_map_yaml(mission_config):
    with open(mission_config, encoding="utf-8") as config_file:
        parameters = yaml.safe_load(config_file)["mission_driver"]["ros__parameters"]
    map_path = parameters.get("navigation_map_yaml", "")
    if not isinstance(map_path, str) or not map_path.strip():
        raise ValueError("Set mission_driver.navigation_map_yaml in mission_config.yaml")
    return os.path.abspath(os.path.expanduser(map_path))


def configured_docked_assumption(mission_config):
    with open(mission_config, encoding="utf-8") as config_file:
        parameters = yaml.safe_load(config_file)["mission_driver"]["ros__parameters"]
    return "true" if parameters.get("assume_docked_on_start", True) else "false"


def generate_launch_description():
    drive_manager_dir = get_package_share_directory("drive_manager")
    rosbridge_launch = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml",
    )
    mission_config = os.path.join(
        drive_manager_dir,
        "param",
        "mission_config.yaml",
    )
    nav2_launch = os.path.join(
        drive_manager_dir,
        "launch",
        "nav2_headless.launch.py",
    )

    start_nav2 = LaunchConfiguration("start_nav2")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")

    return LaunchDescription(
        [
            OpaqueFunction(function=acquire_launch_lock),
            DeclareLaunchArgument(
                "start_nav2",
                default_value="true",
                description="Start Nav2 in lifecycle-inactive mode",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=configured_map_yaml(mission_config),
                description="Full path to the map YAML file used by Nav2",
            ),
            LogInfo(msg=["Navigation map: ", map_yaml_file]),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("use_respawn", default_value="True"),
            DeclareLaunchArgument(
                "assume_docked_on_start",
                default_value=configured_docked_assumption(mission_config),
                description="Assume robot is docked; set false when restarting outside the dock",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                condition=IfCondition(start_nav2),
                launch_arguments={
                    "autostart": "false",
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "use_rviz": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(rosbridge_launch),
            ),
            Node(
                package="drive_manager",
                executable="nav2_supervisor",
                name="nav2_supervisor",
                parameters=[mission_config],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="drive_manager",
                executable="command_manager",
                name="command_manager",
                parameters=[mission_config],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="drive_manager",
                executable="web_teleop",
                name="web_teleop",
                parameters=[mission_config],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="drive_manager",
                executable="mission_driver",
                name="mission_driver",
                parameters=[mission_config, {
                    "navigation_map_yaml": map_yaml_file,
                    "assume_docked_on_start": ParameterValue(
                        LaunchConfiguration("assume_docked_on_start"), value_type=bool,
                    ),
                }],
                output="screen",
            ),
        ]
    )
