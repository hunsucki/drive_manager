import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
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

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(rosbridge_launch),
            ),
            Node(
                package="drive_manager",
                executable="command_manager",
                name="command_manager",
                parameters=[mission_config],
                output="screen",
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
