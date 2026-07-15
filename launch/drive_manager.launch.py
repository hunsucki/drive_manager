import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_nav2",
                default_value="true",
                description="Start Nav2 in lifecycle-inactive mode",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("use_respawn", default_value="True"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                condition=IfCondition(start_nav2),
                launch_arguments={
                    "autostart": "false",
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
                parameters=[mission_config],
                output="screen",
            ),
        ]
    )
